# ESP Driver
## Overview
This repository contains the firmware to run a replacement controller for the SCORBOT-ER 4u. Running on the right hardware, connected to the robot, this program runs in tandem with the Commander application and provides it with a socket connection point through which to control the ER-4's motors.

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/talos-rit/esp-driver)

## Usage
### Hardware Setup
For this program to run, you will need an ESP32 connected to a motor driver module (specific model TBD), four ADS1015 modules for current sensing, and the limit switches of the ER-4. The motors also need to be wired to the motor driver and the current sensors. All of this is detailed in the hardware section of the [Project Documentation](https://github.com/talos-rit/project_documentation) repo.

### Environment and Config
Once the hardware is set up, install either the [ESP-IDF extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension) for VsCode or the standalone ESP-IDF application if you prefer. This guide shows how to use the VsCode extension, but the app should be similar.

Clone and open the repo with the extension enabled. Note that everything in this guide that is described as being done with the toolbar at the bottom of the screen can also be accomplished with commands in an ESP-IDF Terminal, with the syntax `idf.py {command}`. This terminal can be opened through the command palette (Ctrl+Shift+P, then search and select'ESP-IDF: Open ESP-IDF Terminal')

Now for the controller to function it must connect to wifi, so you will need to give it your network ID and password. Use the menuconfig button (gear icon) in the toolbar to open the SDK Config Editor. Open the 'Talos ESP Driver Configuration' section from the list, then find the 'WiFi SSID' and 'WiFi Password' variables on the right. Enter your network ID and password here and save the configuration before exiting.

### Flashing and Running
Connect the ESP32 to your device and make sure your machine can see it. Use the buttons in the toolbar to build the program, and then flash to the detected device. Now use the monitor button to see logging from the program on the controller.

### Connecting Commander
Consult the Commander repo for instructions on running and using the app. Once you're ready to add a connection, you will need the esp-driver's IP address and port for the socket connection. Use the device monitor to look through the programs logs, and if it connected to WiFi successfully there will be a message that says `DRIVER_WIFI: got ip:{IP address}` and another that says `driver_socket: Socket bound on 0.0.0.0:{port}`. These are the values you need to give Commander. The camera address is determined from setup of the [Cam Streamer](https://github.com/talos-rit/cam_streamer) repo, but can be set to your device's webcam with a value of 0 for the purpose of testing Commander - Esp Driver interaction. Now you should be able to input commands from Commander and see it processed by the Esp Driver.

## Testing
### Running Unit Tests
To run unit tests, open an ESP-IDF Terminal and navigate to the `esp-driver\test` directory. Build the project from this directory by running `idf.py build` (this must be done with the CLI rather than with the toolbar button). Once the testing version of the project has been built you can flash the device normally, and then when you monitor the device you should be presented with a testing menu.

## Development Notes
The execution of the program begins with `main\esp_driver.c`'s app_main() function. It is responsible only for initializing the other controller components. No business logic is to be implemented here.

Changing and adding menuconfig variables can be done in `Kconfig.projbuild`.

Problems with including files is likely to be related to improper setup of the related components `CMakeLists.txt` file.