# ESP-EYE Speaker verification
DTW based Text-Dependent Speaker Verification realised for ESP-EYE (can be adapted for ESP32 based boards).  

## Requirements
[**ESP-IDF v3.3.1**](https://github.com/espressif/esp-idf/tree/v3.3.1) is needed before building.  
[ESP-SR](https://github.com/espressif/esp-sr) is included in components.

## Cloning
```
git clone --recursive https://github.com/kasiim/ESP-EYE-speaker-verification.git
```

## Building and flashing
For building use make.
```
make flash
```
or for faster building
```
make -j(number of cpu cores) flash
```

## Usage
When device is programmed, it starts without any stored speakers.  

Program consists of two device modes: Enrollment and verification mode.  
When there are no stored speakers, device starts in enrollment mode. Otherwise device is started in verification mode.

ESP-EYE side button is multifunctional button which provides all controll functions needed.  
Holding the button for 5 seconds in verification mode will change mode to enrollment mode.  
Holding the button for 10 seconds in any mode deletes all stored speakers from system.

System outputs results and information over UART to USB.

Linux users can see output with following command, where in ttyUSBx, x is replaced with device number.
```
screen /dev/ttyUSBx 2000000
```

### Enrollment mode
To enter enrollment mode from verification ESP-EYE side button must be held for 5 seconds.
During enrollment mode, speaker model is created.  
To enroll the phrase, multiple passes must be done.

Enrolling is simple. All that speaker needs to do is push and hold the button, speak the phrase and release the button.  
This action must be repeated until program returns to verification mode.

### Verification mode
During verification mode, device tries to verifiy if phrase that was detected belongs to any enrolled speakers. System detects phrases automatically and only interaction needed is saying the enrolled phrase.

## Config
Config file is located at link: [config.h](main/include/config.h)  
Config is quite self explainatory, audio settings should be left untouched.

## Used libraries
- [UCR Suite](https://www.cs.ucr.edu/~eamonn/UCRsuite.html)
- [KissFFT](https://github.com/mborgerding/kissfft)
- [c_speech_features](https://github.com/Cwiiis/c_speech_features)