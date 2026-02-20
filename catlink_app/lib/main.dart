import 'package:flutter/material.dart';
import 'package:flutter/foundation.dart';
import 'dart:async';
import 'dart:math';
import 'package:synchronized/synchronized.dart';
import 'package:maplibre_gl/maplibre_gl.dart';
import 'package:flutter/services.dart';
import 'package:geolocator/geolocator.dart';
import 'package:permission_handler/permission_handler.dart';
import 'gateway_ble.dart';
import 'info_dialog.dart';
import 'logger.dart';

const double hdopLowAccuracy = 2.0;

const LocationSettings userLocationSettings = LocationSettings(
  accuracy: LocationAccuracy.best,
  distanceFilter: 0,
);
const double defaultZoom = 16.5;
const maptilerKey = String.fromEnvironment('MAPTILER_KEY');

void main() {
  runApp(const _CatLinkDebugApp());
}

class _CatLinkDebugApp extends StatelessWidget {
  const _CatLinkDebugApp();

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: _MapPage());
  }
}

/// Determine the current position of the device.
///
/// When the location services are not enabled or permissions
/// are denied the `Future` will return an error.
Future<Position> _getUserPosition() async {
  bool serviceEnabled;
  LocationPermission permission;

  // Test if location services are enabled.
  serviceEnabled = await Geolocator.isLocationServiceEnabled();
  if (!serviceEnabled) {
    // Location services are not enabled don't continue
    // accessing the position and request users of the
    // App to enable the location services.
    return Future.error('Location services are disabled.');
  }

  permission = await Geolocator.checkPermission();
  if (permission == LocationPermission.denied) {
    permission = await Geolocator.requestPermission();
    if (permission == LocationPermission.denied) {
      // Permissions are denied, next time you could try
      // requesting permissions again (this is also where
      // Android's shouldShowRequestPermissionRationale
      // returned true. According to Android guidelines
      // your App should show an explanatory UI now.
      return Future.error('Location permissions are denied');
    }
  }

  if (permission == LocationPermission.deniedForever) {
    // Permissions are denied forever, handle appropriately.
    return Future.error(
      'Location permissions are permanently denied, we cannot request permissions.',
    );
  }

  // When we reach here, permissions are granted and we can
  // continue accessing the position of the device.
  return await Geolocator.getCurrentPosition();
}

class GatewayConnectButton extends StatefulWidget {
  final GatewayBle _gatewayBle;
  const GatewayConnectButton({super.key, required GatewayBle gatewayBle})
    : _gatewayBle = gatewayBle;

  @override
  State<GatewayConnectButton> createState() => _GatewayConnectButtonState();
}

class _GatewayConnectButtonState extends State<GatewayConnectButton> {
  bool _onGoingRequest = false;

  @override
  Widget build(BuildContext context) {
    return ValueListenableBuilder<GatewayConnState>(
      valueListenable: widget._gatewayBle.gatewayConnState,
      builder: (context, connState, _) {
        final icon = switch (connState) {
          GatewayConnState.disconnected => Stack(
            clipBehavior: Clip.none,
            children: [
              const Icon(Icons.bluetooth_disabled),
              Positioned(
                right: -8,
                top: 14,
                child: Icon(Icons.priority_high, color: Colors.red, size: 20),
              ),
            ],
          ),
          GatewayConnState.connected => Stack(
            clipBehavior: Clip.none,
            children: [
              const Icon(Icons.bluetooth),
              Positioned(
                right: -8,
                top: 14,
                child: Icon(Icons.done_outlined, color: Colors.green, size: 20),
              ),
            ],
          ),
          GatewayConnState.scanning => Stack(
            children: [const CircularProgressIndicator()],
          ),
        };

        _onGoingRequest = false;

        return FloatingActionButton(
          onPressed: () async {
            if (_onGoingRequest) return;

            // Gateway connect/disconnect
            switch (widget._gatewayBle.gatewayConnState.value) {
              case GatewayConnState.disconnected:
                _onGoingRequest = false;
                await widget._gatewayBle.connectToGateway();
                break;
              case GatewayConnState.connected:
                _onGoingRequest = false;
                await widget._gatewayBle.disconnect();
                break;
              case GatewayConnState.scanning:
                break;
            }
          },
          child: icon,
        );
      },
    );
  }
}

class LinkCtrlButton extends StatefulWidget {
  final GatewayBle _gatewayBle;
  const LinkCtrlButton({super.key, required GatewayBle gatewayBle})
    : _gatewayBle = gatewayBle;

  @override
  State<LinkCtrlButton> createState() => _LinkCtrlButtonState();
}

class _LinkCtrlButtonState extends State<LinkCtrlButton> {
  bool _onGoingRequest = false;
  Timer? _scanTimer;
  int _scanCountdown = 42;

  void _startScanCountdown() {
    _scanTimer?.cancel();
    _scanTimer = Timer.periodic(Duration(seconds: 1), (timer) {
      setState(() {
        _scanCountdown--;
      });
      if (_scanCountdown <= 0) {
        timer.cancel();
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return ValueListenableBuilder<LinkCtrlState>(
      valueListenable: widget._gatewayBle.linkCtrlState,
      builder: (context, linkCtrlState, _) {
        // Start countdown when entering scan state
        if (linkCtrlState == LinkCtrlState.scan && _scanTimer == null) {
          _scanCountdown = 42;
          WidgetsBinding.instance.addPostFrameCallback((_) {
            _startScanCountdown();
          });
        } else if (linkCtrlState != LinkCtrlState.scan) {
          _scanTimer?.cancel();
          _scanTimer = null;
        }

        final icon = switch (linkCtrlState) {
          LinkCtrlState.disconnect => Stack(
            clipBehavior: Clip.none,
            children: [const Icon(Icons.wrong_location)],
          ),
          LinkCtrlState.scanTimedout || LinkCtrlState.connectionLost => Stack(
            clipBehavior: Clip.none,
            children: [
              const Icon(Icons.wrong_location),
              Positioned(
                right: -8,
                top: 14,
                child: Icon(Icons.priority_high, color: Colors.red, size: 20),
              ),
            ],
          ),
          LinkCtrlState.connected => Stack(
            clipBehavior: Clip.none,
            children: [
              const Icon(Icons.emergency_share),
              Positioned(
                right: -8,
                top: 14,
                child: Icon(Icons.done_outlined, color: Colors.green, size: 20),
              ),
            ],
          ),
          LinkCtrlState.scan =>
            _scanCountdown > 0
                ? Stack(
                    alignment: Alignment.center,
                    children: [
                      const CircularProgressIndicator(),
                      Text('${_scanCountdown}s'),
                    ],
                  )
                : Stack(children: [const CircularProgressIndicator()]),
        };

        _onGoingRequest = false;

        return FloatingActionButton(
          onPressed: () async {
            if (_onGoingRequest ||
                widget._gatewayBle.gatewayConnState.value !=
                    GatewayConnState.connected) {
              return;
            }

            // Tracker connect/disconnect
            switch (linkCtrlState) {
              case LinkCtrlState.disconnect:
              case LinkCtrlState.scanTimedout:
              case LinkCtrlState.connectionLost:
                _onGoingRequest = true;
                await widget._gatewayBle.scanTracker();
                break;
              case LinkCtrlState.connected:
              case LinkCtrlState.scan:
                _onGoingRequest = true;
                await widget._gatewayBle.disconnectTracker();
                break;
            }
          },
          child: icon,
        );
      },
    );
  }

  @override
  void dispose() {
    _scanTimer?.cancel();
    super.dispose();
  }
}

enum HeartbeatState { disconnected, beating, finishedBeating, connected }

class TrackerHeartbeat extends StatefulWidget {
  final ValueListenable<int> trackerSnr;

  const TrackerHeartbeat({super.key, required this.trackerSnr});

  @override
  State<TrackerHeartbeat> createState() => _TrackerHeartbeatState();
}

class _TrackerHeartbeatState extends State<TrackerHeartbeat>
    with TickerProviderStateMixin {
  Timer? _heartbeatTimer;
  HeartbeatState _heartbeatState = HeartbeatState.disconnected;
  late AnimationController _scaleController;
  late Animation<double> _scale;

  @override
  void initState() {
    super.initState();
    _scaleController = AnimationController(
      duration: const Duration(milliseconds: 2000),
      vsync: this,
    );
    _scale = Tween<double>(begin: 1.0, end: 2.0).animate(
      CurvedAnimation(parent: _scaleController, curve: Curves.easeInOut),
    );
  }

  @override
  Widget build(BuildContext context) {
    return ValueListenableBuilder<int>(
      valueListenable: widget.trackerSnr,
      builder: (context, snr, _) {
        if (_heartbeatState == HeartbeatState.finishedBeating) {
          _heartbeatState = HeartbeatState.connected;
        } else if (snr != disconnectSnr) {
          _heartbeatTimer?.cancel();
          _heartbeatState = HeartbeatState.beating;
          if (!_scaleController.isAnimating) {
            _scaleController.forward().then((_) => _scaleController.reverse());
          }
          _heartbeatTimer = Timer(const Duration(seconds: 1), () {
            if (mounted) {
              setState(() {
                _heartbeatState = HeartbeatState.finishedBeating;
              });
            }
          });
        } else if (snr == disconnectSnr) {
          _heartbeatTimer?.cancel();
          _scaleController.stop();
          _scaleController.reset();
          _heartbeatState = HeartbeatState.disconnected;
        }

        final color = switch (_heartbeatState) {
          HeartbeatState.beating => Colors.blue,
          HeartbeatState.finishedBeating ||
          HeartbeatState.connected => Colors.green,
          HeartbeatState.disconnected => Colors.red,
        };

        return ScaleTransition(
          scale: _scale,
          child: Icon(Icons.favorite, color: color, size: 16.0),
        );
      },
    );
  }

  @override
  void dispose() {
    _scaleController.dispose();
    _heartbeatTimer?.cancel();
    super.dispose();
  }
}

class _MapPage extends StatefulWidget {
  const _MapPage();

  @override
  State<_MapPage> createState() => _MapPageState();
}

class _MapPageState extends State<_MapPage> {
  MapLibreMapController? _controller;
  StreamSubscription<Position>? _userPositionStreamSub;
  late GatewayBle _gatewayBle;
  final _updateLock = Lock();

  // All marker positions
  Map<String, (LatLng, String)> positions = {
    "user": (const LatLng(0, 0), ""),
    "tracker": (const LatLng(0, 0), ""),
  };

  @override
  void initState() {
    super.initState();
    _gatewayBle = GatewayBle();
  }

  Future<void> _loadCustomMarkers() async {
    final ByteData userMarkerBytes = await rootBundle.load(
      'assets/user_marker.png',
    );
    final Uint8List userMarkerImageData = userMarkerBytes.buffer.asUint8List();
    await _controller!.addImage("user-marker", userMarkerImageData);
    final ByteData trackerMarkerBytes = await rootBundle.load(
      'assets/tracker_marker.png',
    );
    final Uint8List trackerMarkerImageData = trackerMarkerBytes.buffer
        .asUint8List();
    await _controller!.addImage("tracker-marker", trackerMarkerImageData);
  }

  Future<void> _initMapSource() async {
    final geojson = _buildGeoJson();

    await _controller!.addSource(
      "map-source",
      GeojsonSourceProperties(data: geojson),
    );

    await _controller!.addLayer(
      "map-source",
      "map-layer",
      const SymbolLayerProperties(
        textField: ["get", "alert"],
        textAnchor: "bottom",
        textOffset: [0, 1.3],
        textSize: 13.0,
        textAllowOverlap: true,
        textIgnorePlacement: true,
        iconImage: "{icon}",
        iconAllowOverlap: true,
        iconIgnorePlacement: true,
      ),
    );
  }

  // Update map source with current positions
  Future<void> _updateMapSource() async {
    if (_controller == null) return;
    await _updateLock.synchronized(() async {
      final geojson = _buildGeoJson();
      await _controller!.setGeoJsonSource("map-source", geojson);
    });
  }

  Map<String, dynamic> _buildGeoJson() {
    return {
      "type": "FeatureCollection",
      "features": positions.entries.map((entry) {
        final (latlng, alert) = entry.value;
        return {
          "type": "Feature",
          "id": entry.key,
          "geometry": {
            "type": "Point",
            "coordinates": [latlng.longitude, latlng.latitude],
          },
          "properties": {
            "icon": entry.key == "user" ? "user-marker" : "tracker-marker",
            "title": entry.key,
            "alert": alert,
          },
        };
      }).toList(),
    };
  }

  Future<void> setTrackerLostConnAlert() async {
    switch (_gatewayBle.linkCtrlState.value) {
      case LinkCtrlState.connectionLost:
        final (latlng, _) = positions["tracker"]!;
        positions["tracker"] = (latlng, "lost conn.");
        await _updateMapSource();
        break;
      default:
        break;
    }
  }

  Future<void> updateTrackerMarker() async {
    final (hasFix, lat, lon, hdop) = _gatewayBle.trackerPosition.value;

    if (!hasFix) {
      final (latlng, _) = positions["tracker"]!;
      positions["tracker"] = (latlng, "no fix");
    } else if (hdop > hdopLowAccuracy) {
      // TODO: accuracy threshold
      positions["tracker"] = (LatLng(lat, lon), "low acc.");
    } else {
      positions["tracker"] = (LatLng(lat, lon), "");
    }
    await _updateMapSource();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        flexibleSpace: Center(
          child: Row(
            mainAxisSize: MainAxisSize.min,
            spacing: 16.0,
            children: [
              Container(
                margin: const EdgeInsets.all(8.0),
                padding: const EdgeInsets.symmetric(
                  horizontal: 12.0,
                  vertical: 8.0,
                ),
                decoration: BoxDecoration(
                  border: Border.all(color: Colors.grey),
                  borderRadius: BorderRadius.circular(8.0),
                ),
                child: Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    const Text('Gateway', style: TextStyle(fontSize: 12.0)),
                    const SizedBox(width: 8.0),
                    ValueListenableBuilder<int>(
                      valueListenable: _gatewayBle.gatewayBattery,
                      builder: (context, batteryLevel, _) {
                        final icon = switch (batteryLevel) {
                          == unknownBattery => Icons.battery_unknown,
                          > 70 => Icons.battery_full,
                          > 60 => Icons.battery_6_bar,
                          > 50 => Icons.battery_5_bar,
                          > 40 => Icons.battery_4_bar,
                          > 30 => Icons.battery_3_bar,
                          > 20 => Icons.battery_2_bar,
                          > 10 => Icons.battery_1_bar,
                          _ => Icons.battery_0_bar,
                        };
                        return Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Icon(icon, size: 20.0),
                            SizedBox(
                              width: 22.0,
                              child: Text(
                                batteryLevel == unknownBattery
                                    ? ''
                                    : '$batteryLevel',
                                style: const TextStyle(fontSize: 11.0),
                              ),
                            ),
                          ],
                        );
                      },
                    ),
                  ],
                ),
              ),
              Container(
                margin: const EdgeInsets.all(8.0),
                padding: const EdgeInsets.symmetric(
                  horizontal: 12.0,
                  vertical: 8.0,
                ),
                decoration: BoxDecoration(
                  border: Border.all(color: Colors.grey),
                  borderRadius: BorderRadius.circular(8.0),
                ),
                child: Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    const Text('Tracker', style: TextStyle(fontSize: 12.0)),
                    const SizedBox(width: 8.0),
                    TrackerHeartbeat(trackerSnr: _gatewayBle.trackerSnr),
                    const SizedBox(width: 8.0),
                    ValueListenableBuilder<int>(
                      valueListenable: _gatewayBle.trackerSnr,
                      builder: (context, snr, _) {
                        final icon = switch (snr) {
                          == disconnectSnr => Icons.signal_cellular_off,
                          < -5 => Icons.signal_cellular_alt_1_bar,
                          < 1 => Icons.signal_cellular_alt_2_bar,
                          _ => Icons.signal_cellular_alt,
                        };
                        return Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Icon(icon, size: 20.0),
                            SizedBox(
                              width: 27.0,
                              child: Text(
                                snr == disconnectSnr ? '' : '${snr}dB',
                                style: const TextStyle(fontSize: 11.0),
                              ),
                            ),
                          ],
                        );
                      },
                    ),
                    const SizedBox(width: 8.0),
                    ValueListenableBuilder<int>(
                      valueListenable: _gatewayBle.trackerBattery,
                      builder: (context, dividedVoltage, _) {
                        logger.d("Divided battery voltage: $dividedVoltage");

                        final double voltage =
                            dividedVoltage.toDouble() / 255.0 * 6.0;
                        double batteryLevel = (voltage - 3.3) / 0.8 * 100.0;
                        batteryLevel = batteryLevel < 0.0 ? 0.0 : batteryLevel;
                        batteryLevel = batteryLevel > 100.0
                            ? 100.0
                            : batteryLevel;

                        IconData icon = switch (batteryLevel) {
                          > 70.0 => Icons.battery_full,
                          > 60.0 => Icons.battery_6_bar,
                          > 50.0 => Icons.battery_5_bar,
                          > 40.0 => Icons.battery_4_bar,
                          > 30.0 => Icons.battery_3_bar,
                          > 20.0 => Icons.battery_2_bar,
                          > 10.0 => Icons.battery_1_bar,
                          _ => Icons.battery_0_bar,
                        };
                        if (dividedVoltage == unknownBattery) {
                          icon = Icons.battery_unknown;
                        }
                        return Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Icon(icon, size: 20.0),
                            SizedBox(
                              width: 22.0,
                              child: Text(
                                dividedVoltage == unknownBattery
                                    ? ''
                                    : '${batteryLevel.round()}',
                                style: const TextStyle(fontSize: 11.0),
                              ),
                            ),
                          ],
                        );
                      },
                    ),
                  ],
                ),
              ),
            ],
          ),
        ),
      ),
      body: MapLibreMap(
        styleString:
            'https://api.maptiler.com/maps/basic-v2/style.json?key=$maptilerKey',
        attributionButtonPosition: AttributionButtonPosition.bottomLeft,
        onMapCreated: (c) async {
          _controller = c;
        },
        onStyleLoadedCallback: () async {
          try {
            await _loadCustomMarkers();

            // Requesting permissions here causes the map to hang
            if (!await Permission.bluetoothScan.isDenied &&
                !await Permission.bluetoothConnect.isDenied &&
                !await Permission.locationWhenInUse.isDenied) {
              _gatewayBle.connectToGateway();
            }

            // Set initial user position
            final Position initialPosition = await _getUserPosition();
            positions["user"] = (
              LatLng(initialPosition.latitude, initialPosition.longitude),
              "",
            );

            // Create initial map source with markers
            await _initMapSource();

            // Initialize camera on user
            final (userLatlng, _) = positions["user"]!;
            await _controller!.moveCamera(
              CameraUpdate.newCameraPosition(
                CameraPosition(target: userLatlng, zoom: defaultZoom),
              ),
            );

            // Set up user location updates
            _userPositionStreamSub =
                Geolocator.getPositionStream(
                  locationSettings: userLocationSettings,
                ).listen((Position positionUpdate) async {
                  positions["user"] = (
                    LatLng(positionUpdate.latitude, positionUpdate.longitude),
                    "",
                  );
                  await _updateMapSource();
                });

            // Set up tracker marker updates
            _gatewayBle.trackerPosition.addListener(updateTrackerMarker);
            _gatewayBle.linkCtrlState.addListener(setTrackerLostConnAlert);
          } catch (e) {
            logger.e("Error in map initialization: $e");
          }
        },
        initialCameraPosition: CameraPosition(
          target: const LatLng(0, 0),
          zoom: defaultZoom,
        ),
      ),
      floatingActionButton: Wrap(
        spacing: 20.0,
        children: [
          FloatingActionButton(
            onPressed: () {
              showInfoDialog(context);
            },
            child: const Icon(Icons.info),
          ),
          GatewayConnectButton(gatewayBle: _gatewayBle),
          LinkCtrlButton(gatewayBle: _gatewayBle),
          FloatingActionButton(
            onPressed: () async {
              // Center between user and tracker
              final (userLatlng, _) = positions["user"]!;
              final (trackerLatlng, _) = positions["tracker"]!;

              final distanceInMeters = Geolocator.distanceBetween(
                userLatlng.latitude,
                userLatlng.longitude,
                trackerLatlng.latitude,
                trackerLatlng.longitude,
              );

              if (distanceInMeters < 300) {
                final midpoint = LatLng(
                  (userLatlng.latitude + trackerLatlng.latitude) / 2,
                  (userLatlng.longitude + trackerLatlng.longitude) / 2,
                );
                await _controller?.animateCamera(
                  CameraUpdate.newCameraPosition(
                    CameraPosition(target: midpoint, zoom: defaultZoom),
                  ),
                );
              } else {
                final bounds = LatLngBounds(
                  southwest: LatLng(
                    min(userLatlng.latitude, trackerLatlng.latitude),
                    min(userLatlng.longitude, trackerLatlng.longitude),
                  ),
                  northeast: LatLng(
                    max(userLatlng.latitude, trackerLatlng.latitude),
                    max(userLatlng.longitude, trackerLatlng.longitude),
                  ),
                );

                await _controller?.animateCamera(
                  CameraUpdate.newLatLngBounds(
                    bounds,
                    left: 100,
                    top: 100,
                    right: 100,
                    bottom: 100,
                  ),
                );
              }
            },
            child: const Icon(Icons.my_location),
          ),
        ],
      ),
    );
  }

  @override
  void dispose() {
    _gatewayBle.linkCtrlState.removeListener(setTrackerLostConnAlert);
    _gatewayBle.trackerPosition.removeListener(updateTrackerMarker);
    _userPositionStreamSub?.cancel();
    _gatewayBle.dispose();
    super.dispose();
  }
}
