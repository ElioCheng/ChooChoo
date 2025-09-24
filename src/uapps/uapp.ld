PHDRS {
    ro PT_LOAD FLAGS(5);
    rw PT_LOAD FLAGS(6);
}

SECTIONS {
    . = 0x400000;  /* User apps start at 4MB where it's user-accessible */

    __text_start = .;
    .text : {
        *(.text.entry)
        *(.text*)
    } :ro
    __text_end = .;

    __rodata_start = .;
    .rodata : {
        *(.rodata*)
    } :ro
    __rodata_end = .;

    __data_start = .;
    .data : {
        *(.data*)
    } :rw
    __data_end = .;

	__bss_start = .;
    .bss : {
        *(.bss*)
    } :rw
    __bss_end = .;
}