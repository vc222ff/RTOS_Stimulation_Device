# 2DT304 - Wearable Posture Correction Device
Members: Victor Cajfeldt<br>
Program: Computer Engineering<br>
Course: 2DT304<br>
Date of latest submission: 2025-03-28<br>

<br>

This README document is divided into three sections: ??? Compilation of java files ???, how to start the TFTP server and the PDF report for the project.

## Instructions for the device

**Table of contents:**<br>
[**1. ????**](#???)<br> 
[**2. ????**](#???)<br> 
[**3. ????**](#???)<br>

*????? ????? ????? ?????*
```ascii
.
├── build
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   ├── cmake_install.cmake
│   ├── compile_commands.json
│   ├── elf2uf2
│   ├── FREERTOS_KERNEL
│   ├── generated
│   ├── Makefile
│   ├── pico-sdk
│   ├── pioasm
│   └── src
├── CMakeLists.txt
├── c_tests
│   └── I2Ctest.c
├── doc
│   ├── img
│   ├── Midterm Presentation.pdf
│   └── pi-pico-w-datasheet.pdf
├── lib
│   ├── FreeRTOS-Kernel
│   └── pico-sdk
├── LICENSE
├── micropython_tests
│   ├── ble_advertising.py
│   ├── ble_uart_peripheral.py
│   ├── Bluetooth_debug.py
│   ├── I2C_loop_test.py
│   ├── I2C_scan.py
│   ├── MicroPythonMain.py
│   ├── PiicoDev_MPU6050.py
│   ├── PiicoDev.py
│   └── PiicoDev_Unified.py
├── pico_sdk_import.cmake
├── react_app
│   ├── app
│   ├── app.json
│   ├── assets
│   ├── components
│   ├── constants
│   ├── expo-env.d.ts
│   ├── node_modules
│   ├── package.json
│   ├── package-lock.json
│   ├── patches
│   ├── services
│   └── tsconfig.json
├── README.md
└── src
    ├── CMakeLists.txt
    ├── FreeRTOSConfig.h
    ├── main2.c
    └── main.c
```
```ascii
.
├── build
├── CMakeLists.txt
├── c_tests
├── doc
├── lib
├── LICENSE
├── micropython_tests
├── pico_sdk_import.cmake
├── react_app
├── README.md
└── src
```

<br><br><br><br><br>








# Instructions for TFTP Server 
Members: Victor Cajfeldt <br>
Program: Computer Engineering<br>
Course: 1DV702<br>
Date of latest submission: 2024-06-09<br>
<br>
This guide is divided into three sections: Compilation of java files, how to start the TFTP server and the PDF report for the project.

**Table of contents:** <br>
[**1. Compilation**](#compilation) <br> 
[**2. Running the TFTP Server**](#running-the-tftp-server)<br> 
[**3. PDF Report**](#pdf-report) <br>


## Compilation
To compile the TFTP server we start by navigating to the root folder, which for this project is the folder "assignment3". We open up the intergrated terminal in the folder and run the following command:<br>


*Compile command:*
```bash 
javac -d bin/ src/TFTPServer.java
```
This command compiles the main class "TFTPServer" and places it in bin/ folder relative from current directory, "assignment3".


*Directory structure after compilation:*
```ascii
.
├── build
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   ├── cmake_install.cmake
│   ├── compile_commands.json
│   ├── elf2uf2
│   ├── FREERTOS_KERNEL
│   ├── generated
│   ├── Makefile
│   ├── pico-sdk
│   ├── pioasm
│   └── src
├── CMakeLists.txt
├── c_tests
│   └── I2Ctest.c
├── doc
│   ├── img
│   ├── Midterm Presentation.pdf
│   └── pi-pico-w-datasheet.pdf
├── lib
│   ├── FreeRTOS-Kernel
│   └── pico-sdk
├── LICENSE
├── micropython_tests
│   ├── ble_advertising.py
│   ├── ble_uart_peripheral.py
│   ├── Bluetooth_debug.py
│   ├── I2C_loop_test.py
│   ├── I2C_scan.py
│   ├── MicroPythonMain.py
│   ├── PiicoDev_MPU6050.py
│   ├── PiicoDev.py
│   └── PiicoDev_Unified.py
├── pico_sdk_import.cmake
├── react_app
│   ├── app
│   ├── app.json
│   ├── assets
│   ├── components
│   ├── constants
│   ├── expo-env.d.ts
│   ├── node_modules
│   ├── package.json
│   ├── package-lock.json
│   ├── patches
│   ├── services
│   └── tsconfig.json
├── README.md
└── src
    ├── CMakeLists.txt
    ├── FreeRTOSConfig.h
    ├── main2.c
    └── main.c
```
After compiling the .java files your project directory will resemble the following structure.


## Running the TFTP server
To start running the Trivial File Transfer Server we navigate to the root directory "assignment3" and open the terminal. Here we execute the following commands to start the TFTP server:


*Simple execution:*
```bash
java -cp .:bin TFTPServer
```
Uses standard settings, port 4970 and paths "public/read" & "public/write".

## PDF Report
The report regarding the project can be found here: 
[PDF report](vc222ff_1dv701_assign3.docx.pdf)

<br><br><br><br>


## Add your files

- [ ] [Create](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#create-a-file) or [upload](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#upload-a-file) files
- [ ] [Add files using the command line](https://docs.gitlab.com/ee/gitlab-basics/add-file.html#add-a-file-using-the-command-line) or push an existing Git repository with the following command:

```
cd existing_repo
git remote add origin https://gitlab.lnu.se/vc222ff/2dt304-wearable-posture-correction-device.git
git branch -M main
git push -uf origin main
```






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
