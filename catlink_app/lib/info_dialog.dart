import 'package:flutter/material.dart';

void showInfoDialog(BuildContext context) {
  showDialog(
    context: context,
    builder: (BuildContext context) {
      return AlertDialog(
        title: const Text('CatLink'),
        content: SingleChildScrollView(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Image.asset('assets/diagram.png'),
              const SizedBox(height: 16),
              const Text(
                "Buttons",
                style: TextStyle(fontWeight: FontWeight.bold),
              ),
              const SizedBox(height: 4),
              _buttonInfoRow(Icons.bluetooth, "Connect/disconnect gateway"),
              const SizedBox(height: 12),
              _buttonInfoRow(
                Icons.location_pin,
                "Connect/disconnect tracker via gateway",
              ),
              const SizedBox(height: 12),
              const Text(
                "Tracker Alerts",
                style: TextStyle(fontWeight: FontWeight.bold),
              ),
              const SizedBox(height: 4),
              _trackerAlertInfoRow(
                "no fix",
                "Tracker is communicating but does not have a position fix yet",
              ),
              const SizedBox(height: 12),
              _trackerAlertInfoRow(
                "low acc.",
                "Tracker is reporting low position accuracy",
              ),
              const SizedBox(height: 12),
              _trackerAlertInfoRow(
                "lost conn.",
                "Connection between tracker and gateway has been lost",
              ),
            ],
          ),
        ),
        actions: [
          TextButton(
            onPressed: () {
              Navigator.of(context).pop();
            },
            child: const Text('Close'),
          ),
        ],
      );
    },
  );
}

Widget _buttonInfoRow(IconData icon, String text) {
  return Row(
    crossAxisAlignment: CrossAxisAlignment.center,
    children: [
      Container(
        padding: const EdgeInsets.all(8),
        decoration: BoxDecoration(
          border: Border.all(color: Colors.grey),
          borderRadius: BorderRadius.circular(4),
        ),
        child: Icon(icon, size: 24),
      ),
      const SizedBox(width: 12),
      Expanded(child: Text(text)),
    ],
  );
}

Widget _trackerAlertInfoRow(String alert, String text) {
  return Row(
    crossAxisAlignment: CrossAxisAlignment.center,
    children: [
      Stack(
        alignment: Alignment.bottomCenter,
        clipBehavior: Clip.none,
        children: [
          Container(
            padding: const EdgeInsets.fromLTRB(14, 8, 14, 16),
            decoration: BoxDecoration(
              border: Border.all(color: Colors.grey),
              borderRadius: BorderRadius.circular(4),
            ),
            child: Icon(Icons.emergency_share, size: 24),
          ),
          Positioned(
            top: 32,
            child: Text(alert, style: const TextStyle(fontSize: 11)),
          ),
        ],
      ),
      const SizedBox(width: 12),
      Expanded(child: Text(text)),
    ],
  );
}
