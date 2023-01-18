# Pioneer-AVIC-F940BT-IR-decoder

Pioneer AVIC-F840BT / F940BT with wired remote input. IR NEC protocol remote is a cheap Pioneer copy from Aliexpress (QXE1047 CXC8885 CXE3669 QXA3196).

ATTiny45 or ATTiny2313 + CD4051 + BC847 + TSOP1738 + 7805...

VCC to ANT out of car audio(+12V when audio is on), GND to GND 3.5mm stereo jack to WR in of car audio.

If you use ATTiny45 (25, 85), you must set the fuse to disable the RESET signal on pin 1,
the only way to program it back with ISP is to use a high voltage programmer to set the fuse to default.

Any ATTiny can be used, just be careful, the input from IR must be on INT0 and the setup of timers must be right (CPU clock, registers)
  
Created by Mellors Baškovič, 12.01.2023.
Released into the public domain.
