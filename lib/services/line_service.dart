import 'dart:convert';
import 'package:http/http.dart' as http;

class LineService {
  final String _channelId = '2010154891';
  final String _channelSecret = '556e653bf5a3e8263760406714a21d8e';

  Future<String?> _getAccessToken() async {
    final url = Uri.parse('https://api.line.me/v2/oauth/accessToken');
    try {
      final response = await http.post(
        url,
        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
        body: {
          'grant_type': 'client_credentials',
          'client_id': _channelId,
          'client_secret': _channelSecret,
        },
      );

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        return data['access_token'];
      } else {
        print(
          'Failed to get LINE access token: ${response.statusCode} - ${response.body}',
        );
        return null;
      }
    } catch (e) {
      print('Error getting LINE access token: $e');
      return null;
    }
  }

  Future<bool> sendEmergencyAlert({
    required String userId,
    double? lat,
    double? lng,
  }) async {
    final token = await _getAccessToken();
    if (token == null) return false;

    final url = Uri.parse('https://api.line.me/v2/bot/message/push');

    String locationText = 'Location unknown';
    if (lat != null && lng != null) {
      locationText = 'https://maps.google.com/?q=$lat,$lng';
    }

    final messageText =
        '''⚠️ Smart Helmet Accident Alert

Possible accident detected.

📍 Location:
$locationText

Please respond immediately.''';

    try {
      final response = await http.post(
        url,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': 'Bearer $token',
        },
        body: json.encode({
          'to': userId,
          'messages': [
            {'type': 'text', 'text': messageText},
          ],
        }),
      );

      if (response.statusCode == 200) {
        return true;
      } else {
        print(
          'Failed to send LINE push message: ${response.statusCode} - ${response.body}',
        );
        return false;
      }
    } catch (e) {
      print('Error sending LINE push message: $e');
      return false;
    }
  }

  Future<bool> sendSafeMessage({required String userId}) async {
    final token = await _getAccessToken();
    if (token == null) return false;

    final url = Uri.parse('https://api.line.me/v2/bot/message/push');

    final messageText =
        ''' Smart Helmet Update: The rider has pressed the safety button on their helmet and marked themselves as SAFE. The previous accident alert can be disregarded.

 頭盔狀態更新：騎士已按下安全帽上的安全按鈕，並標記為「安全」。先前的事故警報可以忽略。''';

    try {
      final response = await http.post(
        url,
        headers: {
          'Content-Type': 'application/json',
          'Authorization': 'Bearer $token',
        },
        body: json.encode({
          'to': userId,
          'messages': [
            {'type': 'text', 'text': messageText},
          ],
        }),
      );

      if (response.statusCode == 200) {
        return true;
      } else {
        print(
          'Failed to send LINE safe message: ${response.statusCode} - ${response.body}',
        );
        return false;
      }
    } catch (e) {
      print('Error sending LINE safe message: $e');
      return false;
    }
  }
}
