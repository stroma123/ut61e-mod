# README #

* UNI-T UT61E modification

The modification adds more features to the function buttons on the UT61E multimeter.
More information about the modification can be found here,
[http://foogadgets.blogspot.com/2015/08/uni-t-ut61e-modification.html](http://foogadgets.blogspot.com/2015/08/uni-t-ut61e-modification.html)

### How do I get set up? ###

This is C-code written for the PIC16F688 microprocessor from Microchip.
I have used Microchip MPLAB XC8 C Compiler V1.34 to compile the code.
The compiler can be downloaded [here](http://www.microchip.com/pagehandler/en_us/devtools/mplabxc/).

Clone the repo and create the directories bin build and obj.
Type "make clean;make all" and program the microprocessor with "make prog".

You will also need pk2cmd (or similar) to program the microprocessor.