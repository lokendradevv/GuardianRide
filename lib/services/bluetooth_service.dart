import 'dart:convert';
import 'dart:async';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import '../providers/app_state.dart';

class HelmetBluetoothService {
  final AppState appState;
  StreamSubscription<List<ScanResult>>? _scanSubscription;
  StreamSubscription<BluetoothConnectionState>? _connectionSubscription;
  BluetoothDevice? _connectedDevice;

  // UUIDs from ESP32 Arduino code
  final String serviceUuid = '4fafc201-1fb5-459e-8fcc-c5c9c331914b';
  final String sensorCharacteristicUuid = 'beb5483e-36e1-4688-b7f5-ea07361b26aa';
  final String dataCharacteristicUuid = 'beb5483e-36e1-4688-b7f5-ea07361b26a8';
  final String ctrlCharacteristicUuid = 'beb5483e-36e1-4688-b7f5-ea07361b26a9';

  HelmetBluetoothService(this.appState);

  Future<void> connectToHelmet() async {
    // Wait for Bluetooth to be turned on
    if (await FlutterBluePlus.adapterState.first != BluetoothAdapterState.on) {
      return;
    }

    try {
      for (var device in FlutterBluePlus.connectedDevices) {
        String name = device.platformName.isNotEmpty ? device.platformName : device.advName;
        if (name.contains('ESP32-S3-CAM') || name.contains('Smart Helmet')) {
          _connect(device);
          return;
        }
      }
    } catch (_) {}

    _scanSubscription = FlutterBluePlus.scanResults.listen((results) async {
      for (ScanResult r in results) {
        // Use both platformName and advName for better Android compatibility
        String deviceName = r.device.platformName.isNotEmpty ? r.device.platformName : r.advertisementData.advName;
        
        if (deviceName.contains('ESP32-S3-CAM') || deviceName.contains('Smart Helmet')) {
          await FlutterBluePlus.stopScan();
          _connect(r.device);
          break;
        }
      }
    });

    await FlutterBluePlus.startScan(timeout: const Duration(seconds: 15));
  }

  Future<void> _connect(BluetoothDevice device) async {
    try {
      // Connect to the device with the required license parameter
      await device.connect(license: License.free);
      _connectedDevice = device;
      appState.setConnectionStatus(true);

      _connectionSubscription = device.connectionState.listen((
        BluetoothConnectionState state,
      ) {
        if (state == BluetoothConnectionState.disconnected) {
          appState.setConnectionStatus(false);
          _connectedDevice = null;
        }
      });

      // Small delay helps Android BLE stack settle before discovering services
      await Future.delayed(const Duration(milliseconds: 600));
      _discoverServices(device);
    } catch (e) {
      print('Connection failed: $e');
    }
  }

  Future<void> _discoverServices(BluetoothDevice device) async {
    List<BluetoothService> services = await device.discoverServices();
    for (var service in services) {
      print('Discovered Service UUID: ${service.uuid}');
      if (service.uuid.toString().toLowerCase() == serviceUuid.toLowerCase()) {
        for (var characteristic in service.characteristics) {
          final uuid = characteristic.uuid.toString().toLowerCase();
          print('Discovered Characteristic UUID: $uuid');

          if (uuid == sensorCharacteristicUuid.toLowerCase()) {
            await characteristic.setNotifyValue(true);
            print('Subscribed to characteristic: $uuid');
            characteristic.onValueReceived.listen((value) {
              print('Received raw bytes length: ${value.length}');
              if (value.isNotEmpty) {
                // Decode string sent from ESP32
                try {
                  String stringValue = utf8.decode(value);
                  print('Decoded string: \$stringValue');
                  // Remove the thermometer emoji and trim
                  stringValue = stringValue.replaceAll('🌡', '').trim();
                  var parts = stringValue.split(' ');
                  
                  double? temp, hum, mag;
                  int? quietMode;
                  String? roadClass;
                  List<int>? pScores;
                  bool? isFall;
                  
                  for (var part in parts) {
                    if (part.startsWith('T:')) {
                      String valStr = part.substring(2).replaceAll('C', '');
                      temp = double.tryParse(valStr);
                    } else if (part.startsWith('H:')) {
                      String valStr = part.substring(2).replaceAll('%', '');
                      hum = double.tryParse(valStr);
                    } else if (part.startsWith('A:')) {
                      var aVals = part.substring(2).split(',');
                      if (aVals.length == 3) {
                        appState.updateIMUData(
                          ax: double.tryParse(aVals[0]),
                          ay: double.tryParse(aVals[1]),
                          az: double.tryParse(aVals[2]),
                        );
                      }
                    } else if (part.startsWith('G:')) {
                      var gVals = part.substring(2).split(',');
                      if (gVals.length == 3) {
                        appState.updateIMUData(
                          gx: double.tryParse(gVals[0]),
                          gy: double.tryParse(gVals[1]),
                          gz: double.tryParse(gVals[2]),
                        );
                      }
                    } else if (part.startsWith('M:')) {
                      String valStr = part.substring(2).replaceAll('g', '');
                      mag = double.tryParse(valStr);
                    } else if (part.startsWith('S:')) {
                      String valStr = part.substring(2);
                      if (valStr == 'CRASH' || valStr == 'IMPACT') isFall = true;
                      else if (valStr == 'OK') isFall = false;
                    } else if (part.startsWith('Q:')) {
                      quietMode = int.tryParse(part.substring(2));
                    } else if (part.startsWith('R:')) {
                      roadClass = part.substring(2);
                    } else if (part.startsWith('P:')) {
                      var pVals = part.substring(2).split(',');
                      if (pVals.length == 4) {
                        pScores = pVals.map((e) => int.tryParse(e) ?? 0).toList();
                      }
                    }
                  }
                  
                  if (temp != null) appState.updateSensorData(temperature: temp);
                  appState.updateTelemetryData(
                    humidity: hum,
                    magnitude: mag,
                    quietMode: quietMode,
                    roadClassification: roadClass,
                    predictionScores: pScores,
                    isFallDetected: isFall,
                  );
                } catch (e) {
                  print("Error parsing sensor data: $e");
                }
              }
            });
          }
        }
      }
    }
  }

  void disconnect() {
    _connectedDevice?.disconnect();
    _scanSubscription?.cancel();
    _connectionSubscription?.cancel();
  }
}
