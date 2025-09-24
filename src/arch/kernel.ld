SECTIONS {
	. = 0x80000;            /* start text at this location */
	__text_start = .;
	.text.boot : {          /* boot code must come first */
		KEEP(*(.text.boot))
	}
	.text : {               /* rest of code */
		KEEP(*(.text*))
	}
	__text_end = .;

	. = 0x200000;           /* start data at this location for MMU setup */
	__rodata_start = .;
	.rodata : {
		KEEP(*(.rodata*))
	}
	__rodata_end = .;

	__data_start = .;
	.data : {
		KEEP(*(.data*))
	}
	__data_end = .;

	__bss_start = .;
	.bss : {
		KEEP(*(.bss*))
	}
	__bss_end = .;

	/* Symbol table section */
	.symbols : {
		__symbols_start = .;
		KEEP(*(.symbols))
		__symbols_end = .;
	}

	. = 0x400000;
	/* User task section */
	.user_task : {
		__user_task_start = .;
		KEEP(*(.user_task))
		__user_task_end = .;
	}

	/* User task stacks */
	. = 0x1000000;
	__user_stacks_start = .;

	/* 64 tasks * 20MB each = 1280MB */
	/* . = . + (64 * 2048 * 1024 * 10); */
	/* __user_stacks_end = .; */

}
