/*
 * stm32f405_resc.ld — Disposition memoire RESC (STM32F405RG, 1 Mo / 192 Ko).
 *
 * Flash : secteur 0 (16 Ko) vecteurs ; secteurs 1-2 (32 Ko) configuration ;
 *         secteurs 3-7 application (jusqu'a 0x0807FFF0) ; 8-10 tampon de mise
 *         a jour ; 11 bootloader. Compatible avec le bootloader de reference.
 * RAM   : SRAM1+2 (128 Ko) : .data, .bss, .ramfunc (chemin FOC), .noinit
 *         CCM (64 Ko)      : etat FOC (.ccmbss) et pile principale
 */
ENTRY(Reset_Handler)

MEMORY
{
    FLASH_VEC (rx)  : ORIGIN = 0x08000000, LENGTH = 16K
    FLASH     (rx)  : ORIGIN = 0x0800C000, LENGTH = 0x73FF0
    RAM       (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
    CCM       (rw)  : ORIGIN = 0x10000000, LENGTH = 64K
}

_estack = ORIGIN(CCM) + LENGTH(CCM);
_min_stack = 0x2000;

SECTIONS
{
    .isr_vector : {
        KEEP(*(.isr_vector))
    } > FLASH_VEC

    .text : {
        . = ALIGN(4);
        *(.text*)
        *(.rodata*)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
    } > FLASH

    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
    .ARM.exidx : {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } > FLASH
    .init_array    : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } > FLASH
    .fini_array    : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } > FLASH

    /* Chemin critique FOC execute depuis la SRAM */
    _siramfunc = LOADADDR(.ramfunc);
    .ramfunc : {
        . = ALIGN(4);
        _sramfunc = .;
        *(.ramfunc*)
        . = ALIGN(4);
        _eramfunc = .;
    } > RAM AT > FLASH

    _sidata = LOADADDR(.data);
    .data : {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    .bss (NOLOAD) : {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    .noinit (NOLOAD) : {
        . = ALIGN(4);
        *(.noinit*)
        . = ALIGN(4);
        end = .;
        _end = .;
    } > RAM

    .ccmbss (NOLOAD) : {
        . = ALIGN(4);
        _sccmbss = .;
        *(.ccmbss*)
        . = ALIGN(4);
        _eccmbss = .;
    } > CCM

    ASSERT(_eccmbss + _min_stack <= _estack, "CCM : plus assez de place pour la pile")

}
