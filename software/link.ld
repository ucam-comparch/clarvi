/*
 * The stack starts at the top of memory, defined by __sp, and grows down.
 * Unfortunately this means we must hardcode the size of memory.
 * Here we choose 64K. Change this value to change the working memory size.
 */
__sp = 0x10000;

SECTIONS
{
    .text.init : {
        *(.text.init)
    }

    .text : {
        *(.text)
    }

    .data : {
        *(.bss)
        *(.sdata)
        *(.rodata)
        *(.data*)
        *(.got*)
    }
}
