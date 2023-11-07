# STM32-HID-H.O.T.A.S
Custom made USB HID H.O.T.A.S. based on STM32F411CEU6 aka Black Pill v2.0  

My 3D models:  
The [hot-swap adapter](https://mmf.io/o/333652).  
Modified [Grip](https://mmf.io/o/334761).

## Up to speed and Attributions
This project was made with CubeMX and built using Make, the Throttle-unit is a separate Arduino mcu which was programmed using ArduinoIDE.

Some 3D prints are community sourced, slightly remixed or fully custom made.  
Gimbal was made by the famous [Olukelo](https://www.thingiverse.com/thing:2496028) with a slightly modded lever arm for easily removable grip (made by [R. D. Beerman](https://github.com/rdbeerman/Joystick-Gimbal)). The original F-16 grip model that I've modified is made by [JFlyer81](https://www.thingiverse.com/thing:4544115).  

My modifications:  
**V1 grip** model is only slightly remixed to fit with my custom made connector which I fitted with a mini-din plug and a matching socket with threads. The V1 model unfortunately lacks rotation in (Z) axis.  
**V2 grip** is heavily modified. I've changed all the screws to use a heated threaded insert instead of directly threading screws into plastic. Not only that, this version added screws on the base of the grip, replacing the one near the expand / FOV button (on pinky), which should make the construction more sturdy. Buttons were changed for a non-printed alternative to make them feel more uniform and the hat-switches will also be changed in the future.

The V2 model is still under development. its main purpose is to make the grip model better by making it compatible with better pushbuttons, 5-way switches and also making it more durable.

## STM32 code walk through
### CubeMX
Most of the code and its structure (by volume) is generated with STM32-CubeMX, this software is responsible for the general setup of all the port io, clock frequency and overall, a mandatory microcontroller setup.  
Once the code is generated, in theory CubeMX does not have to be installed on the system to build the firmware.  
This means that if you're using the same mcu and have soldered the connections according to my schematic, you don't have to install STM32 CubeMX on your system even if you want to change something in my code.  
In case you want to change something fundamental in the mcu setup, or if you're using a different STM32 mcu altogether, you still can convert my project to fit your requirements but for this, you'll be better off installing CubeMX.

### Important project files
Most of the functional code (axis analog readout, gimbal - grip communication protocol and button readings) is available in the [Core/Src/main.c](https://github.com/JustCryen/STM32-HID-H.O.T.A.S/blob/main/HID-Hotas/Core/Src/main.c) file, the other important location containing a custom HID descriptor is located in [USB_DEVICE/App/usbd_custom_hid_if.c](https://github.com/JustCryen/STM32-HID-H.O.T.A.S/blob/main/HID-Hotas/USB_DEVICE/App/usbd_custom_hid_if.c#L93) and starts on line 93.  
This HID descriptor specifies the packet structure and is responsible for transferring all the data from the gimbal to the PC without requiring any separate software to be running in the background. This is possible because USB - HID is an industry standard builtin to the USB ecosystem and every device with a USB port should already be compatible with this communication method.  
This allowed me to build a custom controller that doesn't need any additional software to function and it's compatible with all modern operating systems (so Linux, Windows or Mac OS). **No additional drivers required**.

### Optional addons and bluetooth functionality
Since this is an open source project, basically everything can be added to the code if its output is properly handled on the HID side of things.  
Currently there's only one addon module, an Arduino mcu which is responsible for the Throttle-unit ability to read and send out data to the main STM32 for further processing. It's using Bluetooth as a communication method so it's not really necessary for it to be connected to the same PC at all (although it's also so close that it's kinda silly not to connect it to the PC).  
The only drawback is a noticeable latency between inputs since it's basically an analogue readout, so a Throttle code will probably be rewritten to use a wired connection instead.  
Having Bluetooth functionality is still a "nice to have" feature since adding other wireless peripherals to the H.O.T.A.S. can be really useful to improve versatility and make a perfect setup. For now what comes to mind are peripherals like remotely located button boxes or something wearable to improve immersion.

## Build process and dependencies
To build the project, I've used a Linux machine with `CubeMX` and `make` installed. (additionally make requires `arm-none-eabi-gcc` and `arm-none-eabi-newlib` packages as dependencies to build correctly for this platform)  
Once the code is generated with CubeMX (like this project already is) you can use `make` command in the `HID-Hotas` directory to build the project.  
(Keep in mind that in the Makefile on the line 25, there's a compression option. `-O0` or `-Og` is recommended for debugging reasons, for finished code `-O3` can be used to fully optimize the code.  

I've also installed `stlink` package to use a cheap STlink programmer for flashing the firmware. A simple [shellscript](https://github.com/JustCryen/STM32-HID-H.O.T.A.S/blob/main/HID-Hotas/flash.sh) was created to make firmware flashing easier.

## Debugging
Using Code OSS or vscode, there's a "STlink launch" and "STlink attach" option (Cortex-debug extension needed).  
This would allow you to step through code while the MCU is running and the STlink flashing tool is connected to the microcontroller.


