Off-line testing of the FCU and RMP modules:
fcutest.c, fcu.c, fcu.h
rpmtest.c, radio.c, radio.h
Note that this software uses ncurses which works with Linux (Raspberry Pi) but needs pcurses to work with msys2. 

It is also important to bear in mind:
Both modules use an RS-232C serial interface (FCU at 9600 baud and RMP at 115200 baud), which is independent of the UDP transfers, conforming to the serial interface defined by Elan Informatique in the following manuals:
AIRBUS A320 FCU/EFIS Panel Technical User Manual, Ref. 8056E
AIRBUS A320 Radio Management Panel Technical User Manual, Ref. 8163C, which uses a 32-bit checksum with the transfers.
Both modules (fcu.c and rmp.c) use Posix threads in order to avoid any delay with serial data transfers, which would not apply to UDP transfers.
