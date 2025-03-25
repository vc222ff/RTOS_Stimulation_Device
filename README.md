# 2DT304 - Wearable Posture Correction Device
Members: Victor Cajfeldt<br>
Program: Computer Engineering<br>
Course: 2DT304<br>
Date of latest submission: 2025-04-04<br>

<br>

This README document is divided into four sections: Project structure, Embedded C program: Compilation and start, Mobile App: APK Compilation for android and the PDF report for the program.

## Instructions for the device

**Table of contents:**<br>
[**1. Project Structure**](#project-structure) <br> 
[**2. Embedded C program: Compilation and start**](#installation-and-start-of-embedded-c-program)<br> 
[**3. Mobile App: APK Compilation for Android**](#apk-compilation-of-react-native-application)<br> 
[**4. PDF Report**](#???)<br>


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

<br>

<br>

------




<br><br><br><br><br><br><br><br><br><br>



## Suggestions for a good README
Every project is different, so consider which of these sections apply to yours. The sections used in the template are suggestions for most open source projects. Also keep in mind that while a README can be too long and detailed, too long is better than too short. If you think your README is too long, consider utilizing another form of documentation rather than cutting out information.

## Name
Choose a self-explaining name for your project.

## Description
Let people know what your project can do specifically. Provide context and add a link to any reference visitors might be unfamiliar with. A list of Features or a Background subsection can also be added here. If there are alternatives to your project, this is a good place to list differentiating factors.

## Badges
On some READMEs, you may see small images that convey metadata, such as whether or not all the tests are passing for the project. You can use Shields to add some to your README. Many services also have instructions for adding a badge.

## Visuals
Depending on what you are making, it can be a good idea to include screenshots or even a video (you'll frequently see GIFs rather than actual videos). Tools like ttygif can help, but check out Asciinema for a more sophisticated method.

## Authors and acknowledgment
Show your appreciation to those who have contributed to the project.

## License
For open source projects, say how it is licensed.
