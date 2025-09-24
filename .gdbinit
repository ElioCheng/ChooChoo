set architecture aarch64
set endian little
target remote :1234
file build/kernel.elf
add-symbol-file build/uapp 0x400000