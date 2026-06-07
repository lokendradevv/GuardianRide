
import 'package:url_launcher/url_launcher.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:geolocator/geolocator.dart';
import 'location_service.dart';
import 'line_service.dart';

class EmergencyService {
  final LocationService _locationService = LocationService();
  final LineService _lineService = LineService();

  Future<void> triggerEmergency() async {
    final prefs = await SharedPreferences.getInstance();
    final phone = prefs.getString('emergencyPhone') ?? '';
    final lineUserId = prefs.getString('lineUserId') ?? '';

    // 1. Fetch location
    Position? position = await _locationService.getCurrentLocation();

    // 2. Send LINE Message via Messaging API
    if (lineUserId.isNotEmpty) {
      await _lineService.sendEmergencyAlert(
        userId: lineUserId,
        lat: position?.latitude,
        lng: position?.longitude,
      );
    }

    // 3. Call Emergency Contact
    if (phone.isNotEmpty) {
      final Uri phoneUri = Uri(scheme: 'tel', path: phone);
      if (await canLaunchUrl(phoneUri)) {
        launchUrl(phoneUri); // Fire and forget without await to prevent blocking
      }
    }
  }

  Future<void> sendSafeMessage() async {
    final prefs = await SharedPreferences.getInstance();
    final lineUserId = prefs.getString('lineUserId') ?? '';
    
    if (lineUserId.isNotEmpty) {
      await _lineService.sendSafeMessage(userId: lineUserId);
    }
  }
}
