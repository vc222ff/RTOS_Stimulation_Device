# 2DT304 - Wearable Posture Correction Device
Members: Victor Cajfeldt & Sanaa Abdoulkader<br>
Program: Computer Engineering<br>
Date of latest submission: 2025-MM-DD<br>

This README document is divided into four sections: Project structure, Embedded C program: Compilation and start, Mobile App: APK Compilation for android and the PDF report for the project.

## Instructions for the device

**Table of contents:**<br>
[**1. Project Structure**](#project-structure) <br> 
[**2. Embedded C program: Compilation and start**](#installation-and-start-of-embedded-c-program)<br> 
[**3. Mobile App: APK Compilation for Android**](#apk-compilation-of-react-native-application)<br> 
[**4. PDF Report**](#pdf-report)<br>


## Project structure
The repository is structured and organized in the following way. 
```ascii
.
├── apk_build
├── bin
│   ├── application-v.1.0.2.apk
│   └── c_rtos_posture.uf2
├── c_build
├── CMakeLists.txt
├── doc
├── lib
│   ├── FreeRTOS-Kernel
│   └── pico-sdk
├── pico_sdk_import.cmake
├── react_app
│   ├── app
│   │   ├── +html.tsx
│   │   ├── _layout.tsx
│   │   ├── modal.tsx
│   │   ├── +not-found.tsx
│   │   └── (tabs)
│   │       ├── device.tsx
│   │       ├── index.tsx
│   │       ├── _layout.tsx
│   │       ├── settings.tsx
│   │       └── trends.tsx
│   ├── app.json
│   ├── assets
│   ├── components
│   ├── constants
│   ├── eas.json
│   ├── expo-env.d.ts
│   ├── hooks
│   │   └── usePostureData.ts
│   ├── node_modules
│   ├── package.json
│   ├── package-lock.json
│   ├── patches
│   ├── services
│   │   ├── BLEContext.tsx
│   │   └── BLEService.ts
│   └── tsconfig.json
├── README.md
├── src
│   ├── ble_server.c
│   ├── ble_server.h
│   ├── btstack_config.h
│   ├── CMakeLists.txt
│   ├── flash_storage.c
│   ├── flash_storage.h
│   ├── FreeRTOSConfig.h
│   ├── lwipopts.h
│   ├── main.c
│   └── posture_monitor.gatt
└── test
    ├── c_tests
    │   ├── I2Ctest.c
    │   └── pico_usb_test
    └── micropython_tests
        ├── ble_advertising.py
        ├── ble_uart_peripheral.py
        ├── Bluetooth_debug.py
        ├── I2C_loop_test.py
        ├── I2C_scan.py
        ├── MicroPythonMain.py
        ├── PiicoDev_MPU6050.py
        ├── PiicoDev.py
        └── PiicoDev_Unified.py
```

<br>

## Installation and start of embedded C program 
In order to compile the C project for the Raspberry Pi Pico WH we use the following commands.

```bash
# First we create a c_build directory and cd into it.
mkdir c_build && cd c_build

# Once inside we create a new CMake tools file.
cmake ..

# Then we compile the program using the make command.
make
```

Once the project is compiled without errors, we can find a .uf2 binary file in the path c_build/src/c_rtos_posture.uf2. This file can be transferred to the Raspberry Pi Pico for autonomous execution. 

<br>

## APK compilation of React Native Application
To compile an .apk file for the mobile application we can use the following commmands.

```bash
# Opens React Native subdirectory.
cd react-app

# Runs NPM command for compiling preview version for Android devices.
npm run build_apk
```

Once compilation is done, we can find a QR-code and an URL link in the command line that can be scanned on the android device or entered manually in order to install the mobile app.

<br>

## PDF Report
The PDF report can be found in the following link: 
[PDF report](vc222ff_2dt304_final_report.pdf)
