clear


readfile,raw,gui '..\..\..\kernel\linux-3.6.5\arch\arm\boot\Image'=0x61008000

setreg @R0=0x0
setreg @R1=3000 // machine ID number
setreg @R2=0x0
setreg @PC=0x61008000
