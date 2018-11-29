# Heater-remote

Software to remote control Viessmann heater using KM Bus

This project uses 2 NXP LPC1xxx board to remote control a ViessMann
heater using a KM Bus interface to the heater.

The KM Bus interface is using an NCN5150 ic implemented as
indicated in the datasheet/application note
(https://www.onsemi.com/pub/Collateral/AND9108-D.PDF), and connected
to one UART peripheral of the LPC1xxx chip.

One subproject, SerialRemote, uses an LPC1343 evaluation board to do
UART to USB CDC conversion, plus command interpreting to control the heater.
The other, NetRemote, uses an LPC1769 evaluation board to do UART to
Ethernet/IP conversion, plus command interpreting to control the heater.

Frame receiving and decoding and command parsing of both subprojects are the
same or almost (I first worked on the ethernet one, then switched to the USB
one as the power consumption of that unit was slightly lower).

You can use McuXpresso from NXP to create a project into which you can include
the source code. You will also need the library corresponding to the MCU
used.

In my setup, I already use a ViessMann remote control unit (VITOTROLL 300),
so the code is written to advertise to the heater as a second remote control unit.

The way it work, for controlling the heater is that whenever a command is
received from USB/Ethernet, the program registers itself to the heater as
remote control number 2, then after a few cycles sends the required command
(being either a predefined command, like shutdown, eco mode on, ...) or a
custom KM Bus frame that is passed along the USB/Ethernet. After the command
is sent to the heater, the program then stop responding to the heater, because
it otherwise trigs errors in the heater (which might not have any second
heating circuit onto which a remote control could be registered).

Information for creating this program has been gathered here :
https://github.com/openv/openv/wiki/KM-Bus
and exchanged here
https://github.com/openv/openv/issues/387

This code comes with absolutely no warranty of any kind.
I can not be held responsible for any damage caused by the use/misuse of
this software.