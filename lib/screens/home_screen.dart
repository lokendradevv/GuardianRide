import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/app_state.dart';
import '../services/bluetooth_service.dart';
import 'settings_screen.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Smart Rider Guardian'),
        actions: [
          IconButton(
            icon: const Icon(Icons.settings),
            onPressed: () {
              Navigator.push(
                context,
                MaterialPageRoute(builder: (context) => const SettingsScreen()),
              );
            },
          )
        ],
      ),
      body: Consumer<AppState>(
        builder: (context, appState, child) {
          return Padding(
            padding: const EdgeInsets.all(16.0),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                Expanded(
                  child: ListView(
                    children: [
                      _buildStatusCard(
                        title: 'Connection',
                        value: appState.isConnected ? 'Connected' : 'Disconnected',
                        icon: appState.isConnected ? Icons.bluetooth_connected : Icons.bluetooth_disabled,
                        color: appState.isConnected ? Colors.blue : Colors.grey,
                      ),
                      const SizedBox(height: 16),
                      _buildStatusCard(
                        title: 'Temperature',
                        value: appState.temperature != 0.0 ? '${appState.temperature.toStringAsFixed(1)}°C' : '--°C',
                        icon: Icons.thermostat,
                        color: appState.temperature == 0.0 ? Colors.grey : (appState.temperature >= 25 ? Colors.deepOrange : Colors.lightBlue),
                      ),
                      const SizedBox(height: 16),
                      _buildStatusCard(
                        title: 'Humidity',
                        value: appState.telemetryHumidity != null ? '${appState.telemetryHumidity!.toStringAsFixed(1)}%' : '--%',
                        icon: Icons.water_drop,
                        color: appState.telemetryHumidity == null ? Colors.grey : Colors.blueAccent,
                      ),
                      const SizedBox(height: 16),
                      _buildRainPredictionCard(appState),
                      const SizedBox(height: 16),
                      _buildStatusCard(
                        title: 'IMU Motion Data',
                        value: (appState.accelX == 0.0 && appState.accelY == 0.0 && appState.accelZ == 0.0)
                            ? '--'
                            : 'A: ${appState.accelX.toStringAsFixed(2)} | ${appState.accelY.toStringAsFixed(2)} | ${appState.accelZ.toStringAsFixed(2)}\n'
                              'G: ${appState.gyroX.toStringAsFixed(1)} | ${appState.gyroY.toStringAsFixed(1)} | ${appState.gyroZ.toStringAsFixed(1)}',
                        icon: Icons.screen_rotation,
                        color: Colors.deepPurpleAccent,
                        subtitle: (appState.accelX == 0.0 && appState.accelY == 0.0 && appState.accelZ == 0.0)
                            ? 'Waiting for IMU data'
                            : 'Live Accelerometer & Gyroscope',
                      ),
                      const SizedBox(height: 16),
                      _buildStatusCard(
                        title: 'Magnitude',
                        value: appState.magnitude != null ? '${appState.magnitude!.toStringAsFixed(2)}g' : '--g',
                        icon: Icons.speed,
                        color: appState.magnitude == null ? Colors.grey : Colors.purpleAccent,
                      ),
                      const SizedBox(height: 16),
                      _buildStatusCard(
                        title: 'Safety Status',
                        value: appState.isFallDetected ? 'ACCIDENT' : 'OK',
                        icon: appState.isFallDetected ? Icons.warning : Icons.check_circle,
                        color: appState.isFallDetected ? Colors.red : Colors.green,
                      ),
                      const SizedBox(height: 16),
                      _buildStatusCard(
                        title: 'Quiet Mode',
                        value: appState.quietMode == 1 ? 'ON' : 'OFF',
                        icon: appState.quietMode == 1 ? Icons.volume_off : Icons.volume_up,
                        color: appState.quietMode == 1 ? Colors.grey : Colors.green,
                      ),
                      const SizedBox(height: 16),
                      _buildRoadClassificationCard(appState.roadClassification),
                      const SizedBox(height: 16),
                      _buildFallConditionsCard(appState),
                      const SizedBox(height: 16),
                      _buildPredictionProbabilitiesCard(appState.predictionScores),
                      const SizedBox(height: 16),
                      _buildKNNInfoCard(),
                    ],
                  ),
                ),
                if (!appState.isConnected)
                  ElevatedButton(
                    onPressed: () => appState.bluetoothService.connectToHelmet(),
                    style: ElevatedButton.styleFrom(
                      padding: const EdgeInsets.symmetric(vertical: 16),
                    ),
                    child: const Text('Connect to Helmet', style: TextStyle(fontSize: 18)),
                  ),
              ],
            ),
          );
        },
      ),
    );
  }

  Widget _buildStatusCard({
    required String title,
    required String value,
    required IconData icon,
    required Color color,
    String? subtitle,
  }) {
    return Card(
      elevation: 4,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Row(
          children: [
            Container(
              padding: const EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: color.withValues(alpha: 0.1),
                shape: BoxShape.circle,
              ),
              child: Icon(icon, color: color, size: 32),
            ),
            const SizedBox(width: 20),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    title,
                    style: const TextStyle(
                      fontSize: 16,
                      color: Colors.grey,
                    ),
                  ),
                  const SizedBox(height: 4),
                  FittedBox(
                    fit: BoxFit.scaleDown,
                    alignment: Alignment.centerLeft,
                    child: Text(
                      value,
                      style: TextStyle(
                        fontSize: 24,
                        fontWeight: FontWeight.bold,
                        color: color,
                      ),
                    ),
                  ),
                  if (subtitle != null) ...[
                    const SizedBox(height: 4),
                    Text(
                      subtitle,
                      style: const TextStyle(
                        fontSize: 12,
                        color: Colors.redAccent,
                      ),
                    ),
                  ],
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildRoadClassificationCard(String classification) {
    Color badgeColor;
    switch (classification) {
      case 'regular_road':
        badgeColor = Colors.green;
        break;
      case 'pothole':
        badgeColor = Colors.orange;
        break;
      case 'asphalt_bumps':
        badgeColor = Colors.blue;
        break;
      case 'worn_out_road':
        badgeColor = Colors.red;
        break;
      default:
        badgeColor = Colors.grey;
    }

    return Card(
      elevation: 4,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text('Current Road Classification', style: TextStyle(fontSize: 16, color: Colors.grey)),
            const SizedBox(height: 12),
            Chip(
              label: Text(
                classification.replaceAll('_', ' ').toUpperCase(),
                style: const TextStyle(fontWeight: FontWeight.bold, color: Colors.white),
              ),
              backgroundColor: badgeColor,
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildPredictionProbabilitiesCard(List<int> scores) {
    final classes = ['Asphalt Bumps', 'Pothole', 'Regular Road', 'Worn Out Road'];
    return Card(
      elevation: 4,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text('Prediction Probabilities', style: TextStyle(fontSize: 16, color: Colors.grey)),
            const SizedBox(height: 16),
            ...List.generate(4, (index) {
              int score = scores.length > index ? scores[index] : 0;
              return Padding(
                padding: const EdgeInsets.only(bottom: 12.0),
                child: Row(
                  children: [
                    SizedBox(width: 100, child: Text(classes[index], style: const TextStyle(fontSize: 12))),
                    const SizedBox(width: 8),
                    Expanded(
                      child: ClipRRect(
                        borderRadius: BorderRadius.circular(6),
                        child: LinearProgressIndicator(
                          value: score / 100.0,
                          backgroundColor: Colors.grey.withAlpha(50),
                          color: Colors.blueAccent,
                          minHeight: 12,
                        ),
                      ),
                    ),
                    const SizedBox(width: 8),
                    SizedBox(width: 40, child: Text('$score%', textAlign: TextAlign.right)),
                  ],
                ),
              );
            }),
          ],
        ),
      ),
    );
  }

  Widget _buildFallConditionsCard(AppState appState) {
    double ay = appState.accelY.abs();
    double gx = appState.gyroX.abs();
    double mag = appState.magnitude ?? 1.0;

    bool c1Hit = ay > 1.5;
    bool c2Hit = gx > 100.0;
    bool c3Hit = mag < 0.5;

    // C3 bar uses inverted logic: lower mag = higher bar
    // 0g = 100%, 2g = 0%
    double c3BarVal = (2.0 - mag).clamp(0.0, 2.0) / 2.0;

    Widget buildConditionRow(String label, double value, String unit, String threshold, bool isHit, double progress, Color activeColor, double thresholdRatio) {
      return Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Text(label, style: TextStyle(color: isHit ? Colors.redAccent : Colors.greenAccent, fontWeight: FontWeight.bold)),
              Text(threshold, style: const TextStyle(color: Colors.grey, fontSize: 10)),
              Text('${value.toStringAsFixed(2)} $unit', style: TextStyle(color: isHit ? Colors.redAccent : Colors.white, fontWeight: FontWeight.bold)),
            ],
          ),
          const SizedBox(height: 4),
          LayoutBuilder(
            builder: (context, constraints) {
              return Stack(
                children: [
                  Container(
                    height: 12,
                    decoration: BoxDecoration(
                      color: Colors.grey.withAlpha(50),
                      borderRadius: BorderRadius.circular(6),
                    ),
                  ),
                  Container(
                    height: 12,
                    width: constraints.maxWidth * progress.clamp(0.0, 1.0),
                    decoration: BoxDecoration(
                      color: isHit ? Colors.redAccent : activeColor,
                      borderRadius: BorderRadius.circular(6),
                    ),
                  ),
                  Positioned(
                    left: constraints.maxWidth * thresholdRatio,
                    top: 0,
                    bottom: 0,
                    child: Container(width: 2, color: Colors.white),
                  ),
                ],
              );
            },
          ),
          const SizedBox(height: 16),
        ],
      );
    }

    String statusText = "All clear";
    Color statusColor = Colors.greenAccent;
    if (c1Hit && c2Hit && c3Hit) {
      statusText = "!! ALL CONDITIONS MET !!";
      statusColor = Colors.redAccent;
    } else if (c1Hit || c2Hit || c3Hit) {
      statusText = "Conditions: ${c1Hit ? 'C1 ' : ''}${c2Hit ? 'C2 ' : ''}${c3Hit ? 'C3 ' : ''}active";
      statusColor = Colors.orangeAccent;
    }

    return Card(
      elevation: 4,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text('FALL CONDITIONS', style: TextStyle(fontSize: 16, color: Colors.yellowAccent, fontWeight: FontWeight.bold)),
            const SizedBox(height: 16),
            buildConditionRow('C1 |ay|', ay, 'g', '> 1.5g', c1Hit, ay / 3.0, Colors.orangeAccent, 1.5 / 3.0),
            buildConditionRow('C2 |gyrX|', gx, 'dps', '> 100.0dps', c2Hit, gx / 200.0, Colors.purpleAccent, 100.0 / 200.0),
            buildConditionRow('C3 mag', mag, 'g', '< 0.5g', c3Hit, c3BarVal, Colors.greenAccent, 0.75), // 0.5g is 75% on inverted 0-2g scale
            Text(statusText, style: TextStyle(color: statusColor, fontWeight: FontWeight.bold)),
          ],
        ),
      ),
    );
  }

  Widget _buildKNNInfoCard() {
    return Card(
      elevation: 2,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: const [
            Text('KNN Class Order', style: TextStyle(fontSize: 14, fontWeight: FontWeight.bold, color: Colors.grey)),
            SizedBox(height: 8),
            Text('1. asphalt_bumps\n2. pothole\n3. regular_road\n4. worn_out_road', style: TextStyle(fontSize: 12, color: Colors.grey)),
          ],
        ),
      ),
    );
  }

  Widget _buildRainPredictionCard(AppState appState) {
    if (appState.rain3h == null) {
      return _buildStatusCard(
        title: 'Rain Forecast',
        value: 'Loading...',
        icon: Icons.cloud_download,
        color: Colors.blueGrey,
      );
    }
    
    return Card(
      elevation: 4,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Row(
              children: [
                Icon(Icons.umbrella, color: Colors.blueAccent, size: 24),
                SizedBox(width: 8),
                Text('Rain Prediction (mm)', style: TextStyle(fontSize: 16, color: Colors.grey)),
              ],
            ),
            const SizedBox(height: 16),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: [
                _buildRainColumn('3h', appState.rain3h!),
                _buildRainColumn('6h', appState.rain6h!),
                _buildRainColumn('9h', appState.rain9h!),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildRainColumn(String label, double value) {
    return Column(
      children: [
        Text(label, style: const TextStyle(fontWeight: FontWeight.bold, color: Colors.grey)),
        const SizedBox(height: 8),
        Text(
          value.toStringAsFixed(1),
          style: TextStyle(
            fontSize: 20,
            fontWeight: FontWeight.bold,
            color: value > 0 ? Colors.blueAccent : Colors.grey,
          ),
        ),
      ],
    );
  }
}
