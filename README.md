# ESP32-S3 Audio Streaming via BLE

A working implementation for streaming audio from the Adafruit I2S MEMS Microphone (SPH0645LM4H) to an iPhone over Bluetooth Low Energy (BLE) using the Adafruit ESP32-S3 Feather board.

## Overview

This project solves a challenging integration problem: getting the Adafruit ESP32-S3 Feather to reliably interface with the SPH0645LM4H I2S microphone and stream audio data to an iPhone over BLE. The implementation produces audible audio with minimal static, making it suitable for speech-to-text applications, or playback.

This is one of the few working solutions available for this specific hardware combination, I wasn't able to find a solid solution readily available on the internet.

## Hardware

### Board
- **Adafruit ESP32-S3 Feather** with 4MB Flash, 2MB PSRAM
- STEMMA QT / Qwiic compatible
- Onboard NeoPixel for status indication

### Microphone
- **Adafruit I2S MEMS Microphone Breakout - SPH0645LM4H**
- I2S interface
- **Important**: SEL pin must be tied to 3.3V (enables RIGHT channel output)

## Pin Connections

| ESP32-S3 Pin | Feather Label | Microphone Pin | Description |
|--------------|---------------|----------------|-------------|
| GPIO 5       | 5            | LRCLK/WS       | Word Select / Left-Right Clock |
| GPIO 6       | 6            | BCLK           | Bit Clock |
| GPIO 9       | 9            | DOUT           | Data Output |
| 3.3V         | 3V            | SEL            | Channel Select (tied HIGH for RIGHT channel) |
| GND          | GND           | GND            | Ground |
| 3.3V         | 3V            | 3V             | Power |

## Features

- **BLE Audio Streaming**: Streams 16 kHz PCM audio to iPhone via BLE
- **Start/Stop Control**: iPhone sends START/STOP commands to control recording
- **Ring Buffer Batching**: Implements a ring buffer to batch audio samples and prevent packet drops on iPhone

- **Status Indication**: Uses onboard NeoPixel to show connection and streaming status

## Configuration

### Audio Settings
- **Sample Rate**: 16 kHz (optimized for speech)
- **Bit Depth**: 16-bit PCM (converted from 32-bit I2S)
- **Channels**: Mono (RIGHT channel from microphone)
- **I2S Format**: 32-bit samples, MSB-first, mono mode

### BLE Settings
- **Device Name**: `ESP32S3-Audio`
- **MTU Size**: 247 bytes
- **Service UUID**: `3c3b0001-8c5a-4b78-9c3a-1d5a6db3a001`
- **Control Characteristic** (Write): `3c3b0002-8c5a-4b78-9c3a-1d5a6db3a001`
- **Audio Characteristic** (Notify): `3c3b0003-8c5a-4b78-9c3a-1d5a6db3a001`

### Batching Configuration
- **Samples per Notify**: 120 samples (fits within MTU)
- **Notify Interval**: ~15 ms between notifications
- **Ring Buffer Capacity**: 96,000 samples (~6 seconds of audio)
- **I2S Read Size**: 512 samples per read

## How It Works

### Architecture

The implementation uses a dual-buffer architecture:

1. **I2S Capture Task**: Continuously reads 32-bit samples from the microphone via I2S, converts them to 16-bit PCM, and pushes them into a ring buffer.

2. **BLE Streaming Task**: Pops samples from the ring buffer in batches and sends them as BLE notifications at a controlled rate (~15 ms intervals).

3. **Ring Buffer**: Acts as a safety buffer between capture and transmission, preventing packet drops when BLE transmission is slower than capture rate.

### BLE Protocol

#### Control Commands (Write to Control Characteristic)
- **START**: Begin audio capture and streaming
- **STOP**: Stop audio capture (continues streaming buffered data until ring is empty)

#### Audio Data (Notifications from Audio Characteristic)
Each notification packet contains:
- **Bytes 0-1**: Sequence number (uint16_t, little-endian)
- **Bytes 2-3**: Sample count (uint16_t, little-endian)
- **Bytes 4+**: PCM16 audio samples (int16_t array, little-endian)

### Status Indicators

The onboard NeoPixel indicates system state:
- **Blue**: Waiting for BLE connection / Advertising
- **Green**: Connected, idle
- **Red**: Actively capturing and streaming audio

## Setup Instructions

### Prerequisites

1. **PlatformIO**: I used the PlatformIO extension on VS Code.

3. **iPhone App**: You'll need a BLE client app on iPhone to connect and send START/STOP commands

### Installation

1. Clone or download this project
2. Open in PlatformIO
3. Install dependencies (automatically handled by PlatformIO):
   - `h2zero/NimBLE-Arduino @ ^1.4.2`
   - `adafruit/Adafruit NeoPixel @ ^1.12.3`
4. Connect ESP32-S3 via USB
5. Upload the firmware

### Build Configuration

The project uses the following build flags:
- `DARDUINO_USB_MODE=1`: Force USB device mode for reliable uploads
- `NIMBLE_CFG_INCLUDE_CONFIG_FILE=1`: Use framework-defined NimBLE config

Ensure you copy my platformio.ini file.

### Serial Monitor

The code outputs debug information at 115200 baud. You can monitor:
- BLE connection/disconnection events
- START/STOP command acknowledgments
- I2S read statistics
- BLE transmission statistics
- Ring buffer overflow warnings

## Usage

1. **Power On**: The board will start advertising as "ESP32S3-Audio" (NeoPixel shows blue)

2. **Connect from iPhone**: Use your BLE client app to connect to the device

3. **Subscribe to Audio Characteristic**: Subscribe to notifications on the audio characteristic

4. **Start Recording**: Write "START" (case-insensitive) to the control characteristic
   - NeoPixel turns red
   - I2S capture begins
   - Audio packets start streaming

5. **Stop Recording**: Write "STOP" (case-insensitive) to the control characteristic
   - I2S capture stops
   - Buffered audio continues streaming until ring buffer is empty
   - NeoPixel turns green when finished

## Technical Details

### Why Ring Buffer Batching?

Without batching, I found the iPhone would drop a significant number of packets, resulting in very sped up (chipmunk-like) audio during playback. The ring buffer:
- Decouples I2S capture rate from BLE transmission rate
- Allows batching of samples into larger packets (120 samples per notify)
- Provides a safety margin for temporary BLE transmission delays
- Reduces overhead from frequent small notifications

### Sample Conversion

The microphone outputs 32-bit samples, but we convert to 16-bit PCM for BLE transmission:
- Right-shift by 14 bits to scale from 32-bit to ~18-bit range
- Clamp to int16_t range (-32768 to 32767)
- This preserves audio quality while reducing bandwidth

### I2S Configuration

- Uses ESP-IDF 5.x I2S Standard Driver (STD mode)
- Configured for RIGHT channel only (SEL pin tied HIGH)
- 8 DMA descriptors with 512 samples per frame
- Non-blocking reads with 5ms timeout

## Troubleshooting

### No Audio / Poor Quality
- Ensure your solder connections are good (this caused me problems in the beginning)
- Verify SEL pin is connected to 3.3V (not GND)
- Check all I2S pin connections
- Ensure microphone is powered (3V and GND connected)
- Check serial output for I2S read errors

### Packet Drops on iPhone
- The ring buffer should prevent most drops
- Additonally your iOS app can have a buffer to batch process the incoming BLE notifications (what I did)
- If drops persist, try increasing `RING_CAP_SAMPLES` or `NOTIFY_INTERVAL_MS`
- Monitor `g_droppedSamples` in serial output

### BLE Connection Issues
- Ensure iPhone app has proper BLE permissions
- Check that MTU negotiation succeeds (should be 247)
- Verify UUIDs match between ESP32 and iPhone app

### Static in Audio
- Some static is expected but audio should be clearly audible
- Ensure stable power supply
- Check for loose connections
- Verify I2S clock configuration matches microphone specs

## Limitations

- Some static remains in the audio (acceptable for speech-to-text)
- Ring buffer overflow will drop samples (monitor via serial output)
- Maximum continuous recording limited by ring buffer capacity (~6 seconds buffer)

## License

This project is provided as-is for reference and educational purposes.

## Acknowledgments

This implementation was developed to solve the challenging integration between the Adafruit ESP32-S3 Feather and SPH0645LM4H microphone, as there wasn't a readily available solution online. The ring buffer batching approach was critical for reliable iPhone BLE communication.

