# PKE Meter Replica with EMF Detection

## Project Overview
This project replicates the iconic P.K.E. Meter from the *Ghostbusters* movie, integrating modern electronics to simulate its behavior based on electromagnetic field (EMF) detection. The device uses LEDs, sounds, and servo movement to indicate changes in the EMF environment. This is potentially useful for visualizing, studying, or confirming supernatural or anomalous activities through EMF fluctuations.

## Features
- **EMF Detection**: Utilizes the Adafruit LIS3MDL magnetometer for sensing EMF changes.
- **Display**: ST7789 SPI LCD for graphical representation of EMF data.
- **Audio Feedback**: DFRobot DFPlayer Mini for sound effects.
- **Visual Indicators**: LED sequences display EMF intensity.
- **Movement**: MG90S Servo dynamically responds to EMF readings.
- **Calibration**: Automatic calibration to set a baseline for EMF readings.

## Hardware Requirements
- **Microcontroller**: Teensy 4.0  
- **Display**: ST7789 SPI LCD  
- **Sensor**: Adafruit LIS3MDL (I2C magnetometer)  
- **Audio**: DFRobot DFPlayer Mini  
- **Servo**: MG90S Servo  
- **LEDs**: Various LEDs for visual feedback  
- **Speaker**: For audio output  
- **Power Management**:  
  - 3.3V to 5V power boost module with charging  
  - 3500mAh 3.7V rechargeable battery  
- **Resistors**: 220Ω and 1kΩ for LED control and DFPlayer  

## Software Dependencies
Ensure the following libraries are installed in the Arduino IDE:
- `SPI`  
- `Adafruit_Sensor`  
- `Adafruit_LIS3MDL`  
- `PWMServo`  
- `TFT_eSPI`  
- `CircularBuffer` (latest version)  
- `DFRobotDFPlayerMini`  

**Note**: Ensure all libraries are updated to their latest stable versions unless specified otherwise.

## Setup & Calibration
### 1. Assembly
- Assemble the hardware based on your custom design.
- Ensure all components are securely connected and wired properly.

### 2. Calibration
1. Before calibration, ensure there are no strong magnets or EMF sources nearby.
2. Calibration begins automatically when the device powers on and lasts for about 13 seconds.
3. LEDs and the servo will indicate calibration progress.

### 3. Usage
- Power on the device.  
- After calibration, the device will start detecting EMF changes.  
- **LCD**: Displays EMF levels with color changes based on intensity thresholds.  
- **LEDs**: Light up in sequences at varying rates depending on EMF strength.  
- **Servo**: Moves dynamically in response to EMF readings.  
- **Sounds**: Play when EMF levels cross predefined thresholds.

## Operation
- Hold or place the device in areas where you suspect EMF anomalies.  
- Observe visual and auditory feedback for changes in EMF levels.  

## Known Issues & Limitations
- **Customization**: Code is tailored to the specific parts listed. Changes may be necessary for different hardware configurations.  
- **Complexity**: The build requires precise wiring and component placement.  
- **Sensitivity**: EMF sensitivity may need fine-tuning based on your local environment.

## Contributing
Contributions are welcome! To contribute:
1. Fork the repository.  
2. Create a new branch: `git checkout -b feature-branch`.  
3. Commit your changes: `git commit -m 'Add new feature'`.  
4. Push to your branch: `git push origin feature-branch`.  
5. Open a pull request.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.


## Acknowledgments
- Thanks to the creators of the Arduino libraries used in this project.  
- Inspired by the *Ghostbusters* franchise.
