# Pipe Inspection Bot

## Overview

The Pipe Inspection Bot is an ESP32-based robotic system designed to navigate and inspect pipes. It is equipped with various sensors and components to monitor environmental conditions and provide real-time data.

## Features

- **Remote Control**: The bot can be controlled remotely using a custom-built remote controller.
- **Gas Detection**: Equipped with an MQ135 gas sensor to detect harmful gases.
- **Motor Control**: Dual motor control for precise navigation within pipes.

## Future Enhancements

The following features will be added in future updates:

- **Temperature Sensor**: To monitor the temperature inside the pipes.
- **Humidity Sensor**: To measure the humidity levels.
- **Sonar Sensor**: For obstacle detection and distance measurement.
- **Camera**: For visual inspection and real-time video feed.

## Components

### Remote Controller

- **Buttons**: UP, DOWN, LEFT, RIGHT for navigation.
- **LED Indicators**: Dual-LED for status indication.
- **ESP-NOW Communication**: Wireless communication with the bot.

### Bot

- **Motors**: Dual motors for movement.
- **Gas Sensor**: MQ135 for gas detection.
- **ESP-NOW Communication**: Wireless communication with the remote controller.

## Getting Started

1. **Setup the Remote Controller**: Upload the `Remote.ino` code to the ESP32 on the remote controller.
2. **Setup the Bot**: Upload the `Bot.ino` code to the ESP32 on the bot.
3. **Power On**: Power on both the remote controller and the bot.
4. **Control**: Use the remote controller to navigate the bot and monitor gas levels.

## Code Files

- `Remote/Remote.ino`: Code for the remote controller.
- `Bot/Bot.ino`: Code for the bot.

## License

This project is licensed under the MIT License.