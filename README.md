Play a uncompressed/RGB565 video file read from a microSD card (using SPI) and played on a TFT screen (that uses the ILI9341 display driver).

For interfacing the microSD card, the open-source FatFs driver from ChaN has been used: http://elm-chan.org/fsw/ff/00index_e.html

For the application layer for invoking the FatFs APIs, I ported the example code provided by Mbed to libopencm3. The original Mbed code can be found here: https://os.mbed.com/cookbook/SD-Card-File-System

SPI has limited bandwidth of about 0.5 - 1 MBps and the size of the uncompressed video file was about 40 MB and hence the framerate is really slow. Future updates will have support for MJPEG playback instead so compressed video files can be used and played at a smoother FPS.

![output](https://user-images.githubusercontent.com/7463848/88504308-a535bc00-cfd4-11ea-8d88-3fa69427adc9.gif)

Tested only with the Nucleo F446RE microcontroller, but should work with other STM32F4 Nucleo boards too. For F1/F2/F3 boards, modifications might be needed. 
The libopencm3 folder in this repository is a heavily trimmed version of the original containing the required files needed to compile code only for the STM32F4 platform. 

Run the buildflashscript.sh script in the root folder to quickly flash the image and the source code to your Nucelo board.

Description, images, pin-mappings, scripts, To-Do list etc to be updated.

