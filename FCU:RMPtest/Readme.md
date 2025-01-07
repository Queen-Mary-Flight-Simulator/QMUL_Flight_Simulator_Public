# Off-line Testing of the FCU and RMP Modules

## Source Files
- **FCU Module:**
  - `fcutest.c`
  - `fcu.c`
  - `fcu.h`
- **RMP Module:**
  - `rpmtest.c`
  - `radio.c`
  - `radio.h`

### Compatibility Note
This software uses `ncurses`, which works on Linux systems (e.g., Raspberry Pi). For `msys2` environments, `pcurses` is required instead.

---

## Key Considerations

1. **Serial Interface:**
   - Both modules utilise an RS-232C serial interface:
     - FCU operates at **9600 baud**.
     - RMP operates at **115200 baud**.
   - These interfaces are independent of UDP transfers and conform to the specifications defined by Elan Informatique in the following technical manuals:
     - **AIRBUS A320 FCU/EFIS Panel Technical User Manual** (Ref. 8056E).
     - **AIRBUS A320 Radio Management Panel Technical User Manual** (Ref. 8163C).
   - The RMP module uses a **32-bit checksum** for data transfers.

2. **Multithreading:**
   - Both `fcu.c` and `rmp.c` use **Posix threads** to minimise delays in serial data transfers. 
   - This multithreading approach is not applicable to UDP transfers.

---
