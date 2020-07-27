echo "Exporing gcc-arm path"
export PATH="/opt/gcc-arm/bin:$PATH"
echo "Making libopencm3 library and the src directory files"
make
echo "Generating bin files"
arm-none-eabi-objcopy -Obinary src/sdtfttest.elf src/sdtfttest.bin
echo "Flashing binary to the microcontroller"
st-flash write src/colourlcdtest.bin 0x8000000
