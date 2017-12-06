clear

readfile,raw,gui '..\..\..\kernel\linux\arch\arm\boot\Image'=0x80208000

setreg @R0=0x0
setreg @R1=3000 // machine ID number
setreg @R2=0x0
setreg @PC=0x80208000
