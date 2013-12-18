PIC16F1825 - LED Blink Demo
===========================
#### By: Kevin Killingsworth

### Intro to the [PIC 16F1825][1]:

The [PIC 16F1825][1] is a very low-cost 8-bit microcontroller with a good
amount of RAM, NAND, and even some EEPROM.  Additionally if you use the
low-power version, the 16LF1825, it consumes almost no power during sleep.
It features a highly-configurable internal oscillator which allows you to
bring the clock level down to conserve power as well.

### Demo Summary:

This demo is designed to blink 2 LEDs alternately.

The LEDs positive leads should be connected to the following pins
(Don't forget to hook up a resistor inline!)

    RC0: (Starts in off position)
    RC1: (Starts in on position)

### Prerequisites:

I created this demo using the free versions of the following tools:
 * [Microchip MPLAB X][2] (v1.95)
 * [Microchip XC8 Compiler][3] (v1.21)
 * [MPLAB Code Configurator][4] (installed as plugin within MPLAB X)

You will need to download and install these tools appropriately before you
can open this project and build it.

Also, I use the [Microchip MPLAB ICD3 In-Circuit Debugger][5] to program
the chip and debug it.  I have tried line-by-line debugging and it works
in this configuration.  If you use a different compatible debugger, it should
work just fine as well.

[1]: http://www.microchip.com/wwwproducts/Devices.aspx?dDocName=en546902 "PIC 16F1825"
[2]: http://www.microchip.com/pagehandler/en-us/family/mplabx/ "MPLAB X"
[3]: http://www.microchip.com/pagehandler/en_us/devtools/mplabxc/ "MPLAB XC Compilers"
[4]: http://www.microchip.com/pagehandler/en-us/press-release/microchips-free-code-configura.html "MPLAB Code Configurator"
[5]: http://www.microchip.com/stellent/idcplg?IdcService=SS_GET_PAGE&nodeId=1406&dDocName=en537580 "MPLAB ICD 3 In-Circuit Debugger"


