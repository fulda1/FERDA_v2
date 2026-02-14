# FERDA_v2

FERDA is my version of simple LocoNet® throttle heavily inspired by FREMO FREDi. This version utilize extra cheap MCU CH32V203. Program is developed using Arduino IDE + WCH [Arduino core](https://github.com/openwch/arduino_core_ch32).

![FERDA in box.](https://github.com/fulda1/FERDA_v2/blob/main/hardware/FERDA_v2.jpg)


## Hardware
Ferda is oriented to be effective in network communication. To achieve this, it utilize capacitor for survive disconections (C5). To have little up to date hardware, I decided to use cheap Chinese processor CH32V203K8T6. Used SMD variants of components. I have no collisions on PCB then, and can afford comfortable placement of buttons on PCB in two columns. This variant is very effective as your thumb finger control functions F0 to F4, functions F5 to F8 are little far, but still accessible. Functions F9 to F16 are accessible with shift.

I utilized internal Op Amp in mode of comparator. This comparator is directly connected to Rx pin of USART2. This allow full hardware read of LocoNet messages. For Tx I traditionally utilize software serial based on Timer 2.

![FERDA schematic.](https://github.com/fulda1/FERDA_v2/blob/main/hardware/FERDA_v2_sch.png)

##Software
As mentioned, I use Arduino IDE and [Arduino Core CH32](https://github.com/openwch/arduino_core_ch32). That is little tricky. WCH is not good in support. Then last tagged version 1.0.4 is a little outdated. This application utilize functions available in main (and one extra). It is recommended to:
* install CH32 [Arduino core](https://docs.arduino.cc/learn/starting-guide/cores/)
* download main branch of CH32 core
* copy it ower existing 1.0.4
* On top, you must manually instal [this patch](https://github.com/openwch/arduino_core_ch32/issues/213#issuecomment-3603974284) to have working EEPROM library
* ... and then you can go.

Compilation note - FERDA does not require any MCU speed, it is much effective to run MCU on smallest speed, because it will mean lower consumption. I'm using 48 MHz internal clock.

Program is very primitive, but long. It is re-writen from some old assambler version of code. That is visible in structure.

You can create your own, you are welcome to support this project by your contribution.

## License
You can create your own, You can share, You cannot have any profit, You cannot create any comercials

<img src="https://raw.githubusercontent.com/fulda1/FERDA_v2/refs/heads/main/hardware/cc-icons.svg#cc-logo" alt="CC" width="30" height="30"> <img src="https://raw.githubusercontent.com/fulda1/FERDA_v2/refs/heads/main/hardware/cc-icons.svg#cc-by" alt="BY" width="30" height="30"> <img src="https://raw.githubusercontent.com/fulda1/FERDA_v2/refs/heads/main/hardware/cc-icons.svg#cc-nc" alt="NC" width="30" height="30"> <img src="https://raw.githubusercontent.com/fulda1/FERDA_v2/refs/heads/main/hardware/cc-icons.svg#cc-sa" alt="SA" width="30" height="30">
