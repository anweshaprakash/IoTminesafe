# IoT-Based Mica Miner Safety Helmet

An IoT-enabled smart helmet designed to enhance the safety of mica miners in Jharkhand, India, through real-time environmental monitoring, location tracking, and emergency alert systems. This project was developed as part of an academic IoT Workshop and addresses the pressing need for affordable and scalable safety solutions in unregulated mining conditions.

---

## 👷‍♀️ Problem Statement

Mica miners, especially tribal communities including women and children, work in dangerous environments plagued by:

- Toxic gas accumulation
- Risk of mine collapse and falls
- Extreme temperature and humidity
- Lack of location tracking
- Absence of real-time health monitoring

---

## 🛠️ Proposed System

A wearable helmet-based system featuring:

- **ESP32-WROOM** microcontroller for WiFi communication
- **MQ135** gas sensor for air quality monitoring
- **DHT11** sensor for temperature and humidity
- **MPU6050** accelerometer and gyroscope for fall detection
- **NEO-6M GPS** module for location tracking
- **Blynk IoT** platform for cloud connectivity and real-time monitoring
- **Active Buzzer and LEDs** for visual/audible emergency alerts
- **Lock-In SOS Switch** to send emergency GPS coordinates

---

## 🖥️ Platforms Used

### 🔗 Blynk IoT
- Real-time dashboards for sensor visualization
- Alerts on hazardous gas, fall detection, and SOS
- Map widget for GPS tracking
- Remote buzzer activation

### 💻 PlatformIO with Visual Studio Code
- Used for firmware development on ESP32
- Supports structured code, library management, and debugging
- Integrated Git version control

---

## 📦 Hardware Implementation

- **Power Supply**: 7.4V from 2x Li-ion batteries stepped down using a 7805 regulator to provide a stable 5V
- **Soldering**: Assembled on FR4 Zero PCB using 25W soldering iron, jumper wires, and header pins
- **Mounting**: Circuit mounted on a standard miner helmet using hot glue for shock resistance and compactness

---

## 🔄 System Workflow

1. ESP32 collects sensor data (temperature, gas, motion, GPS)
2. Sends real-time data to Blynk cloud via WiFi
3. Alerts generated if gas thresholds or fall conditions are detected
4. GPS coordinates continuously updated on Blynk dashboard
5. Monitoring authority can trigger buzzer via virtual button
6. Miner can send SOS using physical switch to lock and transmit location

---

## ⚠️ Limitations

- WiFi-only communication (no GSM/LoRa)
- Manual soldering not industrial-grade
- Prototype-level durability and sensor calibration
- GPS unreliable deep underground
- No local display or voice feedback

---

## 🚀 Future Scope

- GSM/LoRa integration for better connectivity
- Rugged waterproof enclosures (IP65+)
- Solar + battery power for off-grid use
- ML-based anomaly detection
- Mesh network for multi-miner tracking
- Emergency services API integration
- Web analytics dashboard for historical data

---


---

## 👩‍💻 Authors

- **Amisha Balwani** 
- **Anwesha Prakash** 

---

## 📚 References

- [ESP32 GPS Tutorial](https://randomnerdtutorials.com/esp32-neo-6m-gps-module-arduino/)
- [MPU6050 Integration](https://randomnerdtutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/)
- [ESP32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf)
- [Project Video Demo](https://www.youtube.com/watch?v=jVSgC2HRkKg)

---

## 📝 License

This project is developed for educational and non-commercial purposes. Feel free to fork and expand upon it with appropriate attribution.


