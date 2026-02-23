import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:permission_handler/permission_handler.dart';
import 'logger.dart';

class ForceNotifier<T> extends ValueNotifier<T> {
  ForceNotifier(super.value);

  void forceNotify() {
    notifyListeners();
  }
}

enum GatewayConnState { disconnected, scanning, connected }

enum LinkCtrlState {
  disconnect(0),
  scan(1),
  connected(2),
  scanTimedout(3),
  connectionLost(4);

  final int value;

  const LinkCtrlState(this.value);
}

const int disconnectSnr = -0x8000000000000000;
const int unknownBattery = -1;
const int bleDelay = 400;

class GatewayBle with WidgetsBindingObserver {
  // Singleton
  static final GatewayBle _instance = GatewayBle._internal();
  factory GatewayBle() => _instance;
  GatewayBle._internal() {
    WidgetsBinding.instance.addObserver(this);
  }

  final FlutterReactiveBle _ble = FlutterReactiveBle();

  // BLE properties
  static final String deviceName = "CatLink";
  static final Uuid catlinkServiceUuid = Uuid.parse(
    "cea35d17-0e25-08b9-14a9-a56b1c535820",
  );
  static final Uuid trackerUpdateCharUuid = Uuid.parse(
    "cea35d17-0e25-08b9-14a9-a56b1c535821",
  );
  static final Uuid linkCtrlCharUuid = Uuid.parse(
    "cea35d17-0e25-08b9-14a9-a56b1c535822",
  );
  static final Uuid batteryServiceUuid = Uuid.parse(
    "0000180f-0000-1000-8000-00805f9b34fb",
  );
  static final Uuid batteryLevelCharUuid = Uuid.parse(
    "00002a19-0000-1000-8000-00805f9b34fb",
  );

  // Background disconnect timer
  Timer? _backgroundTimer;
  static const _backgroundTimeout = Duration(minutes: 4);

  // Gateway connection state for GatewayConnectButton
  final _gatewayConnState = ValueNotifier<GatewayConnState>(
    GatewayConnState.disconnected,
  );
  ValueListenable<GatewayConnState> get gatewayConnState => _gatewayConnState;

  // Gateway link control state for LinkCtrlButton
  final _linkCtrlState = ValueNotifier<LinkCtrlState>(LinkCtrlState.disconnect);
  ValueListenable<LinkCtrlState> get linkCtrlState => _linkCtrlState;

  // Tracker position
  final _trackerPosition = ValueNotifier<(bool, double, double, double)>((
    false,
    0.0,
    0.0,
    -1.0,
  ));
  ValueListenable<(bool, double, double, double)> get trackerPosition =>
      _trackerPosition;

  // Tracker LoRa SNR
  final _trackerSnr = ForceNotifier<int>(disconnectSnr);
  ValueListenable<int> get trackerSnr => _trackerSnr;

  // Tracker battery divided voltage
  final _trackerBattery = ValueNotifier<int>(unknownBattery);
  ValueListenable<int> get trackerBattery => _trackerBattery;

  // Gateway battery level %
  final _gatewayBattery = ValueNotifier<int>(unknownBattery);
  ValueListenable<int> get gatewayBattery => _gatewayBattery;

  // BLE Connection variables
  StreamSubscription<DiscoveredDevice>? _scanStreamSub;
  StreamSubscription<ConnectionStateUpdate>? _gatewayConnStreamSub;
  QualifiedCharacteristic? _linkCtrlChar;
  QualifiedCharacteristic? _trackerUpdateChar;
  QualifiedCharacteristic? _gatewayBatteryChar;
  StreamSubscription<List<int>>? _trackerUpdateIndicate;
  StreamSubscription<List<int>>? _linkCtrlIndicate;

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    switch (state) {
      case AppLifecycleState.paused:
      case AppLifecycleState.inactive:
        _startBackgroundTimer();
        logger.i("App backgrounded");
        break;
      case AppLifecycleState.resumed:
        _cancelBackgroundTimer();
        logger.i("App resumed");
        break;
      default:
        break;
    }
  }

  void _startBackgroundTimer() {
    if (_backgroundTimer != null) return;
    _backgroundTimer = Timer(_backgroundTimeout, () async {
      await disconnect();
    });
  }

  void _cancelBackgroundTimer() {
    _backgroundTimer?.cancel();
    _backgroundTimer = null;
  }

  Future<void> _ensurePermissions() async {
    if (await Permission.bluetoothScan.isDenied) {
      await Permission.bluetoothScan.request();
    }
    if (await Permission.bluetoothConnect.isDenied) {
      await Permission.bluetoothConnect.request();
    }
    if (await Permission.locationWhenInUse.isDenied) {
      await Permission.locationWhenInUse.request();
    }
  }

  Future<bool> connectToGateway({int timeout = 10}) async {
    if (_scanStreamSub != null) {
      logger.w("Already scanning");
      return false;
    }
    if (_gatewayConnStreamSub != null) {
      logger.w("Already connected");
      return false;
    }
    _gatewayConnState.value = GatewayConnState.scanning;
    await _ensurePermissions();
    final completer = Completer<bool>();

    // Start scan
    _scanStreamSub = _ble
        .scanForDevices(
          withServices: [catlinkServiceUuid],
          scanMode: ScanMode.balanced,
        )
        .listen(
          (device) async {
            if (device.name == deviceName && !completer.isCompleted) {
              await _scanStreamSub?.cancel();
              _scanStreamSub = null;
              logger.i("Found device: ${device.name}, connecting...");

              // Connect to device
              _gatewayConnStreamSub = _ble
                  .connectToDevice(
                    id: device.id,
                    connectionTimeout: const Duration(seconds: 8),
                  )
                  .listen(
                    (stateUpdate) async {
                      switch (stateUpdate.connectionState) {
                        case DeviceConnectionState.connected:
                          logger.i("Connected to ${device.name}");
                          _linkCtrlChar = QualifiedCharacteristic(
                            serviceId: catlinkServiceUuid,
                            characteristicId: linkCtrlCharUuid,
                            deviceId: device.id,
                          );
                          _trackerUpdateChar = QualifiedCharacteristic(
                            serviceId: catlinkServiceUuid,
                            characteristicId: trackerUpdateCharUuid,
                            deviceId: device.id,
                          );
                          _gatewayBatteryChar = QualifiedCharacteristic(
                            serviceId: batteryServiceUuid,
                            characteristicId: batteryLevelCharUuid,
                            deviceId: device.id,
                          );

                          await Future.delayed(
                            Duration(milliseconds: bleDelay),
                          );
                          if (!await _subscribeToTrackerUpdate()) {
                            await disconnect();
                            completer.complete(false);
                            return;
                          }
                          await Future.delayed(
                            Duration(milliseconds: bleDelay),
                          );
                          if (!await _subscribeToLinkCtrl()) {
                            await disconnect();
                            completer.complete(false);
                            return;
                          }
                          final linkCtrlData = await _ble.readCharacteristic(
                            _linkCtrlChar!,
                          );
                          if (linkCtrlData.isNotEmpty) {
                            _linkCtrlState.value =
                                LinkCtrlState.values[linkCtrlData[0]];
                            switch (_linkCtrlState.value) {
                              case LinkCtrlState.disconnect:
                              case LinkCtrlState.scanTimedout:
                              case LinkCtrlState.connectionLost:
                                _trackerSnr.value = disconnectSnr;
                                break;
                              default:
                                break;
                            }
                          }
                          final gatewayBatteryLevel = await _ble
                              .readCharacteristic(_gatewayBatteryChar!);
                          if (gatewayBatteryLevel.isNotEmpty) {
                            _gatewayBattery.value = gatewayBatteryLevel[0];
                          }
                          completer.complete(true);
                          _gatewayConnState.value = GatewayConnState.connected;
                          break;

                        case DeviceConnectionState.disconnected:
                          await disconnect();
                          break;

                        default:
                          break;
                      }
                    },
                    onError: (err) {
                      logger.e("Connection error: $err");
                      if (!completer.isCompleted) {
                        _gatewayConnState.value = GatewayConnState.disconnected;
                        completer.completeError(err);
                      }
                    },
                  );
            }
          },
          onError: (err) async {
            logger.w("Scan error: $err");
            if (!completer.isCompleted) {
              await _scanStreamSub?.cancel();
              _scanStreamSub = null;
              _gatewayConnState.value = GatewayConnState.disconnected;
              completer.completeError(err);
            }
          },
        );

    // Auto timeout
    Future.delayed(Duration(seconds: timeout), () async {
      if (!completer.isCompleted) {
        logger.i("Scan timeout");
        await _scanStreamSub?.cancel();
        _scanStreamSub = null;
        _gatewayConnState.value = GatewayConnState.disconnected;
        completer.complete(false);
      }
    });

    return completer.future;
  }

  Future<bool> _subscribeToTrackerUpdate() async {
    try {
      _trackerUpdateIndicate = _ble
          .subscribeToCharacteristic(_trackerUpdateChar!)
          .listen(
            (data) {
              _trackerPosition.value = _extractPositionFromTrackerUpdate(data);
              _trackerSnr.value = data[12] >= 128 ? data[12] - 256 : data[12];
              _trackerSnr.forceNotify();
              if (data[0] & (1 << 0) != 0) {
                _trackerBattery.value = data[1];
              }
            },
            onError: (err) async {
              await disconnect();
              logger.w("Indication error for tracker update: $err");
            },
          );
      logger.i("Subscribed to indications for tracker updates");
      return true;
    } catch (e) {
      logger.e("Subscribe to tracker update error: $e");
      return false;
    }
  }

  Future<bool> _subscribeToLinkCtrl() async {
    try {
      _linkCtrlIndicate = _ble
          .subscribeToCharacteristic(_linkCtrlChar!)
          .listen(
            (data) {
              if (data.isNotEmpty) {
                _linkCtrlState.value = LinkCtrlState.values[data[0]];

                switch (_linkCtrlState.value) {
                  case LinkCtrlState.disconnect:
                  case LinkCtrlState.scanTimedout:
                  case LinkCtrlState.connectionLost:
                    _trackerSnr.value = disconnectSnr;
                    break;
                  default:
                    break;
                }
              }
            },
            onError: (err) async {
              await disconnect();
              logger.w("Indication error for link ctrl: $err");
            },
          );
      logger.i("Subscribed to indications for link ctrl");
      return true;
    } catch (e) {
      logger.e("Subscribe to link ctrl error: $e");
      return false;
    }
  }

  Future<void> disconnect() async {
    logger.i("Disconnecting from gateway");
    await _trackerUpdateIndicate?.cancel();
    _trackerUpdateIndicate = null;

    await Future.delayed(Duration(milliseconds: bleDelay));
    await _linkCtrlIndicate?.cancel();
    _linkCtrlIndicate = null;

    await Future.delayed(Duration(milliseconds: bleDelay));
    await _gatewayConnStreamSub?.cancel();
    _gatewayConnStreamSub = null;
    _linkCtrlChar = null;
    _trackerUpdateChar = null;
    _gatewayBatteryChar = null;
    _gatewayConnState.value = GatewayConnState.disconnected;
  }

  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    _backgroundTimer?.cancel();
    disconnect();
    _gatewayConnState.dispose();
    _linkCtrlState.dispose();
    _trackerPosition.dispose();
    _trackerSnr.dispose();
    _trackerBattery.dispose();
    _gatewayBattery.dispose();
  }

  Future<void> scanTracker() async {
    if (_linkCtrlChar == null) return;
    await _ble.writeCharacteristicWithResponse(_linkCtrlChar!, value: [1]);
  }

  Future<void> disconnectTracker() async {
    if (_linkCtrlChar == null) return;
    await _ble.writeCharacteristicWithResponse(_linkCtrlChar!, value: [0]);
  }

  (bool hasFix, double lat, double lon, double hdop)
  _extractPositionFromTrackerUpdate(List<int> data) {
    final hasFix = (data[0] & (1 << 1)) != 0;
    if (!hasFix) {
      return (hasFix, 0.0, 0.0, 0.0);
    }
    final latHemi = data[10] & (1 << 1);
    final lonHemi = data[10] & (1 << 0);

    final latDms = data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5];
    final latDeg = latDms ~/ 100000000;
    final double latMin = (latDms % 100000000) * 1e-6;
    double lat = latDeg + latMin / 60.0;
    if (latHemi == 0) {
      lat = -lat;
    }

    final lonDms = data[6] << 24 | data[7] << 16 | data[8] << 8 | data[9];
    final lonDeg = lonDms ~/ 10000000;
    final double lonMin = (lonDms % 10000000) * 1e-5;
    double lon = lonDeg + lonMin / 60.0;
    if (lonHemi == 0) {
      lon = -lon;
    }

    final double hdop = data[11] * 1e-1;

    logger.d("Fix: $hasFix, Lat: $lat, Lon: $lon, Hdop: $hdop");
    return (hasFix, lat, lon, hdop);
  }
}
