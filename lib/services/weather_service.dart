import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:geolocator/geolocator.dart';
import 'location_service.dart';

class WeatherService {
  final String _apiKey = '38cae8e557b2dc0b2a7f77a36a7e37c3';
  final LocationService _locationService = LocationService();

  Future<({double feelsLike, double humidity, double rain3h, double rain6h, double rain9h})?> fetchEnvironmentData() async {
    try {
      Position? position = await _locationService.getCurrentLocation();
      if (position == null) {
        throw Exception("Location denied or disabled");
      }

      final weatherUrl = Uri.parse(
        'https://api.openweathermap.org/data/2.5/weather?lat=${position.latitude}&lon=${position.longitude}&appid=$_apiKey&units=metric',
      );
      final forecastUrl = Uri.parse(
        'https://api.openweathermap.org/data/2.5/forecast?lat=${position.latitude}&lon=${position.longitude}&appid=$_apiKey&units=metric',
      );

      final responses = await Future.wait([
        http.get(weatherUrl),
        http.get(forecastUrl),
      ]);

      if (responses[0].statusCode == 200 && responses[1].statusCode == 200) {
        final weatherData = json.decode(responses[0].body);
        final forecastData = json.decode(responses[1].body);
        
        return (
          feelsLike: (weatherData['main']['feels_like'] as num).toDouble(),
          humidity: (weatherData['main']['humidity'] as num).toDouble(),
          rain3h: (forecastData['list'][0]['pop'] as num).toDouble() * 100, // +3 hours
          rain6h: (forecastData['list'][1]['pop'] as num).toDouble() * 100, // +6 hours
          rain9h: (forecastData['list'][2]['pop'] as num).toDouble() * 100, // +9 hours
        );
      } else {
        throw Exception("API Error W:${responses[0].statusCode} F:${responses[1].statusCode}");
      }
    } catch (e) {
      rethrow;
    }
  }
}
