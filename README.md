# Gravity Defyers 🤖⚖️

**Team 4 - Self-Balancing Robot Challenge**
**SRM Team Robotics - Interdomain Recruitment Project**

---

## 🏆 The Challenge

Build a two-wheeled robot that dares to defy gravity. The moment it powers on, it has one mission: fight to stay upright. Every fraction of a second becomes a high-speed feedback loop of sensing, calculating, and adjusting.

But real success isn't about balancing in perfect conditions—it's about resilience. Your bot must recover from a push, absorb a bump, and adapt instantly when its load changes.

### The Final Test
Picture carefully placing a full cup of coffee on top of your robot. If it can hold steady—without spilling a drop—you've achieved victory.

---

## 🎓 Project Context

This project was developed as part of the **SRM Team Robocon Interdomain Recruitment Process**. It serves as a comprehensive assessment of mechanical design, embedded programming, control systems, and teamwork skills essential for competitive robotics.

---

## 👥 Team Members

| Team Name | Members |
|-----------|---------|
| **SPACED** | Shivam Jha, Dhruv Shukla |
| **SAMBED** | Rijul, Anshumaan Singh |
| **SIESED** | Lohith Bala, D.J. Einstein |
| **MCSOCD** | Soham Das, Venkatanathan |

---

## 🎯 Core Directives

- **Sense**: Use sensors to continuously detect tilt and motion, giving the robot constant awareness of its balance state
- **React**: Implement a control system that translates sensor data into rapid, precise motor adjustments
- **Endure**: Withstand pushes, bumps, and surface irregularities without losing balance

---

## 🛠️ Hardware Components

### Required Components
- **Microcontroller**: ESP32 Development Board
- **IMU Sensor**: MPU6050 (3-axis accelerometer + 3-axis gyroscope)
- **Stepper Motors**: 2x NEMA17 17HS4401D Stepper Motors
- **Motor Drivers**: 2x TB6600 Stepper Motor Drivers
- **Power Supply**: 12V Battery Pack
- **Chassis**: Custom-built frame (low center of gravity)
- **Wheels**: 2x wheels with good traction

### Component Specifications
- **ESP32**: 32-bit dual-core processor, Wi-Fi & Bluetooth enabled
- **MPU6050**: 6-DOF IMU with 16-bit ADC, I2C interface
- **NEMA17 17HS4401D**: 1.7A, 1.8° step angle, 40mm length
- **TB6600**: Microstep driver, 0.2-5A current range, 9-42V input
- **12V Battery**: Recommended capacity: 2200mAh+ for extended operation

### Pin Configuration
```
MPU6050 IMU:
- SDA → ESP32 GPIO 21
- SCL → ESP32 GPIO 22
- VCC → 3.3V
- GND → GND

TB6600 Motor Driver (Right):
- PUL+ → ESP32 GPIO 32
- DIR+ → ESP32 GPIO 33
- PUL-, DIR- → ESP32 GND
- VCC → 12V Battery +
- GND → 12V Battery -
- A+, A-, B+, B- → NEMA17 Right Motor

TB6600 Motor Driver (Left):
- PUL+ → ESP32 GPIO 25
- DIR+ → ESP32 GPIO 26
- PUL-, DIR- → ESP32 GND
- VCC → 12V Battery +
- GND → 12V Battery -
- A+, A-, B+, B- → NEMA17 Left Motor

Power Connections:
- 12V Battery + → TB6600 VCC (both drivers)
- 12V Battery - → TB6600 GND & ESP32 GND (common ground)
- ESP32 VIN → 12V Battery + (through voltage regulator if needed)
```

### TB6600 Driver Settings
- **Current Setting**: Set to appropriate value for 17HS4401D (1.7A)
- **Microstep Resolution**: Recommended 1/8 or 1/16 for smooth operation
- **Decay Mode**: Fast decay for better torque

---

## 💻 Software Architecture

### Core Algorithm: Complementary Filter + PID Control

1. **Sensor Fusion**: Combines accelerometer and gyroscope data using complementary filter
2. **Balance Detection**: Continuously calculates tilt angle
3. **PID Controller**: Generates motor commands to maintain balance
4. **Motor Control**: Drives stepper motors with precise timing

### Key Features
- **200Hz Control Loop**: Fast response for stable balancing
- **Gyroscope Calibration**: Automatic offset compensation on startup
- **Complementary Filter**: 96% gyro, 4% accelerometer for optimal angle estimation
- **PID Control**: Proportional-Integral-Derivative controller for smooth corrections
- **Integral Windup Protection**: Prevents controller saturation

---

## 📊 Current Implementation Status

### ✅ Implemented Features
- [x] MPU6050 sensor initialization and calibration
- [x] Complementary filter for angle calculation
- [x] Basic PID controller framework
- [x] Stepper motor control interface
- [x] Serial debugging output

### ⚠️ Known Issues & Improvements Needed
1. **Motor Control**: Current implementation sends single pulses instead of continuous pulse trains
2. **Timing**: Blocking delays interfere with control loop consistency
3. **PID Tuning**: Default values need optimization for specific hardware
4. **Direction Logic**: May need reversal based on mechanical setup

---

## 🔧 Setup Instructions

### 1. Hardware Assembly
1. **Mount IMU sensor** on the chassis (ensure proper orientation - Y-axis along pitch)
2. **Install NEMA17 stepper motors** with proper mechanical coupling to wheels
3. **Configure TB6600 drivers**: Set current limit and microstep resolution via DIP switches
4. **Wire according to pin configuration** above - pay attention to common ground
5. **Mount 12V battery pack** with easy access for charging
6. **Balance mechanical design**: Ensure center of mass is above wheel axle
7. **Test all connections** before powering on - verify voltage levels

### 2. Software Installation
1. Install Arduino IDE or PlatformIO
2. Install ESP32 board package
3. Install required libraries:
   - `MPU6050` library by Electronic Cats
   - `Wire` library (built-in)
4. Configure TB6600 drivers (current setting and microstep resolution)
5. Upload the code to ESP32
6. Open Serial Monitor (115200 baud)

### 3. Calibration Process
1. **Power on sequence**: TB6600 drivers first, then ESP32
2. Place robot on **perfectly flat surface**
3. Keep IMU **completely still** during startup calibration
4. Wait for "Gyro Offsets" message in serial output
5. Robot is now ready for balance testing
6. **Safety**: Have manual power cutoff ready during testing

---

## 🎛️ PID Tuning Guide

### Initial Settings (Current)
```cpp
float Kp = 1.0;   // Proportional gain
float Ki = 0.0;   // Integral gain  
float Kd = 0.0;   // Derivative gain
```

### Recommended Tuning Process
1. **Start with P-only**: Set `Kp = 15`, others to 0
2. **Test oscillation**: Robot should swing around balance point
3. **Add D control**: Gradually increase `Kd` (try 0.5, 1.0, 2.0) to reduce oscillations
4. **Add I control**: Only if steady-state error exists (try 0.1, 0.5)

### Tuning Tips
- **Too much P**: Fast oscillations, unstable
- **Too little P**: Slow response, falls over
- **Too much D**: Jittery, high-frequency noise
- **Too much I**: Overshooting, slow settling

---

## 🐛 Debugging

### Serial Monitor Output
The code outputs real-time debugging information:
- **Error**: Difference between target and current angle
- **Angle**: Current tilt angle from accelerometer
- **PID Value**: Controller output
- **Speed**: Mapped motor speed value

### Common Issues
1. **Robot falls immediately**: 
   - Check PID gains, increase Kp
   - Verify TB6600 current settings for NEMA17
   - Ensure 12V power supply is adequate
2. **Continuous oscillation**: 
   - Reduce Kp or increase Kd
   - Check mechanical backlash in wheel coupling
3. **Slow response**: 
   - Increase Kp
   - Verify TB6600 microstep settings (lower = faster)
4. **Wrong direction**: 
   - Check motor wiring sequence (A+, A-, B+, B-)
   - Verify DIR pin logic in code
5. **Sensor drift**: 
   - Recalibrate gyroscope offsets
   - Check MPU6050 mounting and vibrations
6. **Motor stuttering**:
   - Check TB6600 power connections
   - Verify common ground between ESP32 and drivers
   - Ensure adequate 12V current supply

---

## 📈 Performance Metrics

### Target Specifications
- **Control Frequency**: 200 Hz (5ms loop time)
- **Balance Accuracy**: ±2 degrees
- **Recovery Time**: <1 second after disturbance
- **Coffee Test**: Hold full cup without spilling

### Current Performance
- **Loop Time**: ~5ms (needs optimization)
- **Balance Range**: TBD (requires testing)
- **Recovery**: TBD (requires testing)

---

## 🚀 Future Improvements

### Immediate Priorities
1. Fix motor control timing issues
2. Optimize PID parameters
3. Implement proper step generation
4. Add motor speed ramping

### Advanced Features
- **Bluetooth/WiFi Control**: Remote monitoring and tuning
- **Data Logging**: Record balance performance
- **Auto-tuning**: Adaptive PID parameter adjustment
- **Position Control**: Move to specific locations while balancing
- **Advanced Filtering**: Kalman filter implementation

---

## 📚 References & Resources

### Technical Documentation
- [ESP32 Official Documentation]([https://docs.espressif.com/projects/esp32/en/latest/](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp-dev-kits-en-master-esp32.pdf))
- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [TB6600 Stepper Driver Manual]([http://www.leadshine.com/UploadFile/Down/TB6600pg20111019.pdf](https://www.handsontec.com/dataspecs/TB6600-Motor-Driver.pdf))
- [NEMA17 17HS4401D Specifications]([https://www.omc-stepperonline.com/nema-17-stepper-motor-26ncm-36-8oz-in-1-7a-42x42x40mm-4-wires.html](https://www.alldatasheet.com/datasheet-pdf/view/1131976/MOTIONKING/17HS4401.html))
- [PID Control Theory](https://en.wikipedia.org/wiki/PID_controller)

### SRM Team Robocon Resources
- [SRM Team Robocon Official Website]([https://srmrobocon.com](https://www.srmteamrobocon.com/))
- Self-balancing robot theory and implementations
- Stepper motor control with TB6600 drivers
- ESP32 programming for robotics applications

---

## 📝 License

This project is open source.

---

## 🤝 Contributing

Feel free to submit issues, fork the repository, and create pull requests for any improvements.

---

**Built with ❤️ by Team 4 - Gravity Defyers**  
**SRM Team Robocon Interdomain Recruitment 2025**

*"The only way to achieve the impossible is to believe it is possible."*

---

## 🏆 Acknowledgments

Special thanks to **SRM Team Robocon** for providing this challenging and educational project opportunity. This interdomain task has enhanced our understanding of:
- Control systems and PID tuning
- Embedded programming with ESP32
- Stepper motor control and driver integration
- Sensor fusion and IMU applications
- Collaborative robotics development

**SRM Team Robocon** - Robotics Reimagined
