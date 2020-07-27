export PATH="/opt/gcc-arm/bin:$PATH"
arm-none-eabi-objcopy -Obinary sdtfttest.elf sdtfttest.bin
st-flash write sdtfttest.bin 0x8000000
