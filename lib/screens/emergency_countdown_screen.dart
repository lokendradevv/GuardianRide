import 'dart:async';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/app_state.dart';
import '../services/emergency_service.dart';

class EmergencyCountdownScreen extends StatefulWidget {
  const EmergencyCountdownScreen({super.key});

  @override
  State<EmergencyCountdownScreen> createState() => _EmergencyCountdownScreenState();
}

class _EmergencyCountdownScreenState extends State<EmergencyCountdownScreen> {
  int _secondsRemaining = 10;
  Timer? _timer;
  bool _isSending = false;
  final EmergencyService _emergencyService = EmergencyService();

  @override
  void initState() {
    super.initState();
    _startTimer();
  }

  void _startTimer() {
    _timer = Timer.periodic(const Duration(seconds: 1), (timer) {
      if (!mounted) {
        timer.cancel();
        return;
      }
      setState(() {
        if (_secondsRemaining > 0) {
          _secondsRemaining--;
        } else {
          timer.cancel();
          _isSending = true;
          _triggerEmergency();
        }
      });
    });
  }

  Future<void> _triggerEmergency() async {
    await _emergencyService.triggerEmergency();
    if (mounted) {
      final appState = Provider.of<AppState>(context, listen: false);
      appState.markEmergencySent();
      appState.cancelEmergency();
    }
  }

  void _cancelEmergency() {
    if (_isSending) return;
    _timer?.cancel();
    Provider.of<AppState>(context, listen: false).cancelEmergency();
  }

  @override
  void dispose() {
    _timer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.red.shade900,
      body: SafeArea(
        child: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              const Icon(
                Icons.warning_amber_rounded,
                size: 100,
                color: Colors.white,
              ),
              const SizedBox(height: 24),
              const Text(
                'EMERGENCY DETECTED!',
                textAlign: TextAlign.center,
                style: TextStyle(
                  fontSize: 32,
                  fontWeight: FontWeight.bold,
                  color: Colors.white,
                ),
              ),
              const SizedBox(height: 16),
              const Text(
                'Calling emergency contact in...',
                style: TextStyle(
                  fontSize: 18,
                  color: Colors.white70,
                ),
              ),
              const SizedBox(height: 40),
              if (_isSending)
                const CircularProgressIndicator(color: Colors.white)
              else
                Text(
                  '$_secondsRemaining',
                  style: const TextStyle(
                    fontSize: 120,
                    fontWeight: FontWeight.bold,
                    color: Colors.white,
                  ),
                ),
              const SizedBox(height: 60),
              if (!_isSending)
                ElevatedButton(
                  onPressed: _cancelEmergency,
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.white,
                    foregroundColor: Colors.red.shade900,
                    padding: const EdgeInsets.symmetric(horizontal: 48, vertical: 20),
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(30),
                    ),
                  ),
                  child: const Text(
                    'CANCEL',
                    style: TextStyle(
                      fontSize: 24,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                ),
            ],
          ),
        ),
      ),
    );
  }
}
