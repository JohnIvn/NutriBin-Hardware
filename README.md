# NutriBin — Machine Hardware

<img width="936" height="328" alt="image" src="https://github.com/user-attachments/assets/6c962171-3add-41db-a3ba-0d2597b2c2d6" />

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE) [![Project Status](https://img.shields.io/badge/status-Experimental-orange.svg)](#)

## Project Structure

The project is organized into modular sketches within the `Hardware/` directory:

- **[ESP_CAM](Hardware/ESP_CAM/ESP_CAM.ino)**: MJPEG Stream Server for real-time visual monitoring using the ESP32-CAM module.
- **[ESP_AP](Hardware/ESP_AP/ESP_AP.ino)**: Configures the ESP32 as an Access Point (SSID: `Nutribin_Ap`).
- **[ESP_OTA](Hardware/ESP_OTA/ESP_OTA.ino)**: WiFi Manager integration for easy provisioning and connection management.
- **ARDUINO_R3 / ARDUINO_Q**: (Planned) Core logic and sensor processing for Arduino-compatible controllers.
- **ESP_WS**: (Planned) WebSocket communication for real-time telemetry.

## Hardware Requirements

- **ESP32-CAM** (AI-Thinker model)
- **ESP32 Dev Module**
- **Arduino Uno R3** (or compatible)
- Sensors/Actuators (Specifics depending on the implementation)

## Getting Started

### Prerequisites

1.  Standard **Arduino IDE** or **VS Code** with the Arduino extension.
2.  **ESP32 Board Support**:
    - Add `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json` to your Additional Boards Manager URLs.
    - Install `esp32` via the Boards Manager.
3.  **Libraries**:
    - `WiFiManager` (for ESP_OTA)

### Flashing Firmware

1.  Open the desired sketch (e.g., `Hardware/ESP_CAM/ESP_CAM.ino`).
2.  Select your board (e.g., `AI Thinker ESP32-CAM` for the camera, or `ESP32 Dev Module`).
3.  Configure your WiFi credentials in the sketch (if not using WiFiManager).
4.  Connect your device and click **Upload**.

## Usage

### ESP32-CAM Monitoring

Once flashed, the ESP32-CAM will host a web server.

1. Connect to the same WiFi network.
2. Navigate to the ESP32's IP address in your browser.
3. The stream is available at `http://<IP_ADDRESS>/stream`.

### WiFi Management (ESP_OTA)

1. If no credentials are saved, the ESP32 will start an AP named `MyDevice-Setup`.
2. Connect to it via smartphone/PC to configure your local WiFi network.

## Contributing

Please refer to [.github/CONTRIBUTING.md](.github/CONTRIBUTING.md) for contribution guidelines.

## License

See [LICENSE](LICENSE) for details.
