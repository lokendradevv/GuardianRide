import 'package:flutter/foundation.dart';
import 'package:vibration/vibration.dart';
import 'package:audioplayers/audioplayers.dart';
import 'dart:math';
import '../services/weather_service.dart';
import '../services/bluetooth_service.dart';
import '../services/emergency_service.dart';

class AppState extends ChangeNotifier {
  bool _isConnected = false;
  double _temperature = 0.0;
  double? _envFeelsLike;
  double? _envHumidity;
  double? _rain3h;
  double? _rain6h;
  double? _rain9h;
  bool _isHelmetWorn = false;
  bool _isFallDetected = false;
  bool _isEmergencyCountdownActive = false;
  bool _isHeatDangerAcknowledged = false;
  bool _isGyroWarningActive = false;

  final AudioPlayer _audioPlayer = AudioPlayer();

  final WeatherService _weatherService = WeatherService();
  late final HelmetBluetoothService bluetoothService;

  AppState() {
    bluetoothService = HelmetBluetoothService(this);
    fetchEnvironmentTemperature();
  }

  String? _envWeatherError;

  bool get isConnected => _isConnected;
  double get temperature => _temperature;
  double? get envFeelsLike => _envFeelsLike;
  double? get envHumidity => _envHumidity;
  double? get rain3h => _rain3h;
  double? get rain6h => _rain6h;
  double? get rain9h => _rain9h;
  bool get isHeatDanger {
    if (_envFeelsLike != null && _envHumidity != null) {
      double risk = (_envFeelsLike! / 50) + (_envHumidity! / 100);
      // Increased threshold from 1.2 to 1.5 to prevent false triggers on normal days
      return risk > 1.5;
    }
    return false;
  }

  String? get envWeatherError => _envWeatherError;
  bool get isHelmetWorn => _isHelmetWorn;
  bool get isFallDetected => _isFallDetected;
  bool get isEmergencyCountdownActive => _isEmergencyCountdownActive;

  void setConnectionStatus(bool status) {
    _isConnected = status;
    notifyListeners();
  }

  void updateSensorData({
    double? temperature,
    bool? isHelmetWorn,
    bool? isFallDetected,
  }) {
    if (temperature != null) _temperature = temperature;
    if (isHelmetWorn != null) _isHelmetWorn = isHelmetWorn;
    if (isFallDetected != null) {
      _handleFallDetectionState(isFallDetected);
    }
    notifyListeners();
  }

  // IMU Data
  double _accelX = 0.0, _accelY = 0.0, _accelZ = 0.0;
  double _gyroX = 0.0, _gyroY = 0.0, _gyroZ = 0.0;

  double get accelX => _accelX;
  double get accelY => _accelY;
  double get accelZ => _accelZ;
  double get gyroX => _gyroX;
  double get gyroY => _gyroY;
  double get gyroZ => _gyroZ;

  // New ESP32-S3-CAM Data
  double? _telemetryHumidity;
  double? _magnitude;
  int _quietMode = 0;
  String _roadClassification = 'Unknown';
  List<int> _predictionScores = [0, 0, 0, 0];

  double? get telemetryHumidity => _telemetryHumidity;
  double? get magnitude => _magnitude;
  int get quietMode => _quietMode;
  String get roadClassification => _roadClassification;
  List<int> get predictionScores => _predictionScores;

  void updateTelemetryData({
    double? humidity,
    double? magnitude,
    int? quietMode,
    String? roadClassification,
    List<int>? predictionScores,
    bool? isFallDetected,
  }) {
    if (humidity != null) _telemetryHumidity = humidity;
    if (magnitude != null) _magnitude = magnitude;
    if (quietMode != null) _quietMode = quietMode;
    if (roadClassification != null) _roadClassification = roadClassification;
    if (predictionScores != null && predictionScores.length == 4) {
      _predictionScores = predictionScores;
    }

    if (isFallDetected != null) {
      _handleFallDetectionState(isFallDetected);
    }
    
    _checkFallConditions();
    notifyListeners();
  }

  void updateIMUData({
    double? ax,
    double? ay,
    double? az,
    double? gx,
    double? gy,
    double? gz,
  }) {
    if (ax != null) _accelX = ax;
    if (ay != null) _accelY = ay;
    if (az != null) _accelZ = az;
    if (gx != null) _gyroX = gx;
    if (gy != null) _gyroY = gy;
    if (gz != null) _gyroZ = gz;
    
    _checkFallConditions();
    notifyListeners();
  }

  Future<void> _triggerGyroWarning() async {
    try {
      bool? hasVibrator = await Vibration.hasVibrator();
      if (hasVibrator == true) {
        Vibration.vibrate(pattern: [500, 1000, 500, 1000]);
      }

      await _audioPlayer.play(AssetSource('audio/help_me.mp3'));

      // Wait for the audio to completely finish playing
      await _audioPlayer.onPlayerComplete.first;
    } catch (e) {
      print("Warning trigger error: \$e");
    } finally {
      // Unlock the state so it can trigger again if needed
      _isGyroWarningActive = false;
    }
  }

  Future<void> fetchEnvironmentTemperature() async {
    _envWeatherError = null;
    Future.microtask(notifyListeners);
    try {
      final weatherData = await _weatherService.fetchEnvironmentData();
      if (weatherData != null) {
        _envFeelsLike = weatherData.feelsLike;
        _envHumidity = weatherData.humidity;
        _rain3h = weatherData.rain3h;
        _rain6h = weatherData.rain6h;
        _rain9h = weatherData.rain9h;
        // Disabled Heat Danger auto-countdown to prevent false "alarms" when the helmet is stationary
        if (isHeatDanger) {
          _isHeatDangerAcknowledged = false;
        }
      }
    } catch (e) {
      _envWeatherError = e.toString().replaceAll('Exception: ', '');
    }
    notifyListeners();
  }

  void cancelEmergency() {
    _isEmergencyCountdownActive = false;
    _ignoreCrashUntilOK = true; // Ignore further CRASH until OK is received
    
    if (isHeatDanger) {
      _isHeatDangerAcknowledged = true;
    }

    // Stop the audio immediately if the user cancels the emergency
    if (_isGyroWarningActive) {
      _audioPlayer.stop();
      _isGyroWarningActive = false;
    }

    notifyListeners();
  }

  bool _ignoreCrashUntilOK = false;
  bool _hasSentEmergencyMessage = false;

  void markEmergencySent() {
    _hasSentEmergencyMessage = true;
    notifyListeners();
  }

  void _handleFallDetectionState(bool isFall) {
    if (isFall == false) {
      _isFallDetected = false;
      _ignoreCrashUntilOK = false;
      if (_isEmergencyCountdownActive) {
        cancelEmergency(); // Stops sounds and resets state
      }
      if (_hasSentEmergencyMessage) {
        _hasSentEmergencyMessage = false;
        EmergencyService().sendSafeMessage();
      }
    } else if (isFall == true) {
      _isFallDetected = true;
      if (!_ignoreCrashUntilOK && !_isEmergencyCountdownActive) {
        _isEmergencyCountdownActive = true;
      }
    }
  }

  DateTime? _lastC1Time;
  DateTime? _lastC2Time;
  DateTime? _lastC3Time;
  
  void _checkFallConditions() {
    if (_isFallDetected || _ignoreCrashUntilOK) return;
    
    final now = DateTime.now();
    
    // Clear any stale timestamps that are older than 600ms to prevent ghost accumulation
    if (_lastC1Time != null && now.difference(_lastC1Time!).inMilliseconds > 600) _lastC1Time = null;
    if (_lastC2Time != null && now.difference(_lastC2Time!).inMilliseconds > 600) _lastC2Time = null;
    if (_lastC3Time != null && now.difference(_lastC3Time!).inMilliseconds > 600) _lastC3Time = null;
    
    // Condition 1: lateral accel |ay| > 1.5 g
    if (_accelY.abs() > 1.5) _lastC1Time = now;
    
    // Condition 2: roll rate |gyrX| > 100 °/s
    if (_gyroX.abs() > 100.0) _lastC2Time = now;
    
    // Condition 3: magnitude drop mag < 0.5 g
    if (_magnitude != null && _magnitude! < 0.5) _lastC3Time = now;

    if (_lastC1Time != null && _lastC2Time != null && _lastC3Time != null) {
      final t1 = _lastC1Time!.millisecondsSinceEpoch;
      final t2 = _lastC2Time!.millisecondsSinceEpoch;
      final t3 = _lastC3Time!.millisecondsSinceEpoch;

      final maxTime = [t1, t2, t3].reduce(max);
      final minTime = [t1, t2, t3].reduce(min);

      if (maxTime - minTime <= 600) {
        _isFallDetected = true;
        _isEmergencyCountdownActive = true;
        
        // Trigger the siren warning
        if (!_isGyroWarningActive) {
          _isGyroWarningActive = true;
          _triggerGyroWarning();
        }
        
        _lastC1Time = null;
        _lastC2Time = null;
        _lastC3Time = null;
      }
    }
  }
}
