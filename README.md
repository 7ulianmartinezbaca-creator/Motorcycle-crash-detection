# Motorcycle Proximity & Crash Detection System  
A Smart IoT Safety Device for Riders to help motorcylist be noticed better at intercection 

## Overview  
This project is an IoT safety system designed to improve rider awareness and provide automatic emergency notification in the event of a crash. The device uses multiple sensors to detect when vehicles approach too closely at a stop light, alert the motorcyclist through visual cues, and monitor impact forces. All sensor data is published to an Adafruit IO dashboard for real-time tracking and logging.

## Features  
- **Proximity Detection with Ultrasonic Sensor**  
  Measures the distance between the motorcycle and any approaching vehicle when stopped.  
  - **Green NeoPixel Animation:** Safe following distance  
  - **Red NeoPixel Alert:** Vehicle is too close

- **Crash Detection with MPU6080**  
  Monitors sudden shocks, impact forces, and extreme changes that indicate a crash event.

- **GPS Tracking**  
  Streams real-time location updates to Adafruit IO to support incident tracking, emergency response, and post-event review.

- **Cloud Dashboard (Adafruit IO)**  
  All sensor metrics are published to a live dashboard, including:  
  - Proximity distance  
  - Impact/shock readings  
  - GPS coordinates  
  - System status indicators  

## Hardware Used  
- Ultrasonic Distance Sensor (HC-SR04 or equivalent)  
- MPU6080 (Accel/Gyro module)  
- NeoPixel LED strip or ring  
- GPS Module  
- Microcontroller (ESP32, Feather, Photon 2, etc.)  
- Battery power supply

## How It Works  
1. **Distance Monitoring**  
   When the motorcycle is stopped, the ultrasonic sensor continuously measures the distance behind the rider. If an approaching car enters the unsafe zone, the NeoPixels switch to a red alert state. If the distance is safe, the LEDs remain green.

2. **Crash Sensing**  
   The MPU6080 registers sudden, high-G shocks or rapid changes in orientation. If these values exceed a defined threshold, the system flags a potential crash.

3. **Data Upload**  
   Distance, impact readings, and GPS coordinates are transmitted to Adafruit IO for real-time visualization and logging.

4. **Event Awareness**  
   The system provides immediate awareness for the rider and enables remote visibility for family or emergency services.

## Use Case  
Rear-end collisions at intersections are a major risk for motorcyclists. This project provides real-time spatial awareness and automated crash detection without requiring any rider input.

## Future Improvements  
- Automatic SMS/911 crash alerting  
- Mobile app integration  
- Additional LED patterns for increased night visibility  
- Vibration/haptic feedback  
- On-bike audible warnings

## Dashboard Data  
Your Adafruit IO dashboard can track:  
- Distance (cm)  
- Shock/impact data  
- GPS map  
- Event timestamps  

## License  
MIT License 
