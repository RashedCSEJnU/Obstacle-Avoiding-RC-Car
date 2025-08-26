# 🚗 Obstacle-Avoiding RC Car

An intelligent remote-controlled car that combines manual Bluetooth control with autonomous obstacle avoidance capabilities using ultrasonic sensing technology.

## 🌟 Features

- **Dual Control Modes**: Manual Bluetooth control and autonomous obstacle avoidance
- **Smart Navigation**: Uses HC-SR04 ultrasonic sensor for distance measurement
- **Servo-based Scanning**: 180-degree environmental scanning for optimal path planning
- **Real-time Decision Making**: Intelligent obstacle detection and avoidance algorithms
- **Wireless Control**: Bluetooth connectivity for remote operation

## 🔧 Hardware Components

### Core Components

- **Microcontroller**: Arduino (compatible board)
- **Motors**: Dual DC motors for differential drive
- **Distance Sensor**: HC-SR04 Ultrasonic Sensor
- **Servo Motor**: SG90 or similar for sensor scanning
- **Wireless**: Bluetooth module (HC-05/HC-06)
- **Power**: Battery pack suitable for motors and electronics

### Pin Configuration

| Component           | Arduino Pin    |
| ------------------- | -------------- |
| Left Motor Forward  | Digital Pin 5  |
| Left Motor Reverse  | Digital Pin 4  |
| Right Motor Forward | Digital Pin 7  |
| Right Motor Reverse | Digital Pin 6  |
| Ultrasonic Trigger  | Digital Pin 9  |
| Ultrasonic Echo     | Digital Pin 8  |
| Servo Motor         | Digital Pin 10 |
| Bluetooth RX        | Digital Pin 11 |
| Bluetooth TX        | Digital Pin 12 |

## 🚀 Getting Started

### Prerequisites

- Arduino IDE installed
- Basic understanding of Arduino programming
- Bluetooth terminal app on smartphone (like Serial Bluetooth Terminal)

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/RashedCSEJnU/Obstacle-Avoiding-RC-Car.git
   ```
2. Open `code.txt` in Arduino IDE
3. Connect your Arduino board
4. Upload the code to your Arduino

### Usage

#### Manual Control Commands

Send these commands via Bluetooth:

- `F` - Move Forward (with obstacle detection)
- `B` - Move Backward
- `L` - Turn Left
- `R` - Turn Right
- `S` - Stop

#### Autonomous Mode

When moving forward (`F` command), the car automatically:

- Scans for obstacles using the ultrasonic sensor
- Stops when obstacle detected (< 2cm)
- Performs 180° environmental scan
- Backs up and turns left to avoid obstacle
- Continues forward movement

## 🎮 Control Interface

The car responds to single-character commands sent via Bluetooth. Use any Bluetooth terminal application to send commands and control the vehicle remotely.

## 📊 How It Works

1. **Distance Measurement**: Ultrasonic sensor continuously measures forward distance
2. **Decision Logic**: If distance > 3cm, continue forward; if < 2cm, execute avoidance
3. **Avoidance Sequence**: Stop → Scan environment → Reverse → Turn → Continue
4. **Manual Override**: Direct control commands override autonomous behavior

## 🛠️ Customization

- Adjust distance thresholds in the code for different sensitivity levels
- Modify delay timings for different movement speeds
- Customize avoidance patterns by changing the turn sequences

## 📸 Project Gallery

Check out the included photos and video demonstration of the car in action!

## 🤝 Contributing

Feel free to fork this project and submit pull requests for improvements. Some ideas for enhancement:

- Add more sophisticated pathfinding algorithms
- Implement PID control for smoother movements
- Add LED indicators for status
- Include speed control functionality

## 📄 License

This project is open source and available under the [MIT License](LICENSE).

## 👤 Author

**Rashed CSE JnU**

- GitHub: [@RashedCSEJnU](https://github.com/RashedCSEJnU)

## 🙏 Acknowledgments

- Thanks to the Arduino community for extensive documentation
- Inspired by robotics enthusiasts worldwide
- Special thanks to all contributors and testers

---

⭐ **Star this repository if you found it helpful!** ⭐
