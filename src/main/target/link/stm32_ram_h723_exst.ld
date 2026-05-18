/*
*****************************************************************************
**
**  File        : stm32_ram_h723_exst.ld
**
**  Abstract    : Linker script for STM32H723xG / STM32H725xG EXST targets.
**                Firmware runs in-place from OCTOSPI external flash
**                (memory-mapped mode at 0x90100000). Bootloader enables
**                OCTOSPI MM, ISR vectors are relocated to DTCM at startup.
**
**                320K AXI-SRAM on D1 domain
**                 16K SRAM1 on D2 domain
**                 16K SRAM2 on D2 domain
**                 16K SRAM4 on D3 domain
**                 64K ITCM
**                128K DTCM
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/*
0x00000000 to 0x0000FFFF   64K ITCM
0x20000000 to 0x2001FFFF  128K DTCM — stack, data, ISR vector copy
0x24000000 to 0x2404FFFF  320K AXI SRAM, D1 domain — main RAM
0x30000000 to 0x30007FFF   32K D2 SRAM (SRAM1 16K + SRAM2 16K)
0x38000000 to 0x38003FFF   16K SRAM4, D3 domain
0x38800000 to 0x38800FFF    4K BACKUP SRAM
0x70000000               256M OCTOSPI2 (external flash, memory-mapped)
0x90000000               256M OCTOSPI1 (external flash, memory-mapped)
0x90100000 + 1M - 64B    CODE: firmware image (hard-coded, do not change)
*/

/* EXST hash — 64 bytes appended at end of firmware image */
_exst_hash_size = 64;

/* Specify the memory areas */
MEMORY
{
    ITCM_RAM (rwx)      : ORIGIN = 0x00000000, LENGTH = 64K
    DTCM_RAM (rwx)      : ORIGIN = 0x20000000, LENGTH = 128K
    RAM (rwx)           : ORIGIN = 0x24000000, LENGTH = 320K

    D2_RAM (rwx)        : ORIGIN = 0x30000000, LENGTH = 32K   /* SRAM1 16K + SRAM2 16K */
    D3_RAM (rwx)        : ORIGIN = 0x38000000, LENGTH = 16K
    BACKUP_SRAM (rwx)   : ORIGIN = 0x38800000, LENGTH = 4K

    MEMORY_B1 (rx)      : ORIGIN = 0x60000000, LENGTH = 0K

    OCTOSPI2 (rx)       : ORIGIN = 0x70000000, LENGTH = 256M
    OCTOSPI1 (rx)       : ORIGIN = 0x90000000, LENGTH = 256M
    OCTOSPI1_CODE (rx)  : ORIGIN = 0x90000000 + 1M, LENGTH = 1M - 64  /* fixed by bootloader */
    EXST_HASH (rx)      : ORIGIN = 0x90000000 + 1M + (1M - 64), LENGTH = 64
}

REGION_ALIAS("STACKRAM", DTCM_RAM)
REGION_ALIAS("FASTRAM",  DTCM_RAM)
REGION_ALIAS("MAIN",     OCTOSPI1_CODE)
REGION_ALIAS("VECTAB",   MAIN)

/* Highest address of the user mode stack */
_estack = ORIGIN(STACKRAM) + LENGTH(STACKRAM);

__octospi1_start = ORIGIN(OCTOSPI1);
__octospi2_start = ORIGIN(OCTOSPI2);

_Min_Heap_Size  = 0;
_Min_Stack_Size = 0x800;

/* Define output sections */
SECTIONS
{
  /* ISR vector table — LMA in OCTOSPI (MAIN), VMA at VECTAB */
  _isr_vector_table_flash_base = LOADADDR(.isr_vector);
  PROVIDE (isr_vector_table_flash_base = _isr_vector_table_flash_base);

  .isr_vector :
  {
    . = ALIGN(512);
    PROVIDE (isr_vector_table_base = .);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
    PROVIDE (isr_vector_table_end = .);
  } >VECTAB AT> MAIN

  /* ISR vector copy in DTCM — startup relocates VTOR here for fast dispatch */
  _ram_isr_vector_table_base = LOADADDR(.ram_isr_vector);
  PROVIDE (ram_isr_vector_table_base = _ram_isr_vector_table_base);

  .ram_isr_vector (NOLOAD) :
  {
    . = ALIGN(512);
    PROVIDE (ram_isr_vector_table_base = .);
    . += (isr_vector_table_end - isr_vector_table_base);
    . = ALIGN(4);
    PROVIDE (ram_isr_vector_table_end = .);
  } >DTCM_RAM

  /* Code and read-only data execute from OCTOSPI (memory-mapped) */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.rodata)
    *(.rodata*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >MAIN

  /* Critical ISR / loop code in ITCM — zero-latency, loaded from OCTOSPI at startup */
  tcm_code = LOADADDR(.tcm_code);
  .tcm_code :
  {
    . = ALIGN(4);
    tcm_code_start = .;
    *(.tcm_code)
    *(.tcm_code*)
    . = ALIGN(4);
    tcm_code_end = .;
  } >ITCM_RAM AT >MAIN

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >MAIN
  .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >MAIN

  .pg_registry :
  {
    PROVIDE_HIDDEN (__pg_registry_start = .);
    KEEP (*(.pg_registry))
    KEEP (*(SORT(.pg_registry.*)))
    PROVIDE_HIDDEN (__pg_registry_end = .);
  } >MAIN

  .pg_resetdata :
  {
    PROVIDE_HIDDEN (__pg_resetdata_start = .);
    KEEP (*(.pg_resetdata))
    PROVIDE_HIDDEN (__pg_resetdata_end = .);
  } >MAIN

  /* Initialised .data — stored in MAIN (OCTOSPI), copied to DTCM at startup */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >DTCM_RAM AT >MAIN

  /* Non-critical code placed in AXI SRAM for faster execution than OCTOSPI */
  ram_code = LOADADDR(.ram_code);
  .ram_code :
  {
    . = ALIGN(4);
    ram_code_start = .;
    *(.ram_code)
    *(.ram_code*)
    . = ALIGN(4);
    ram_code_end = .;
  } >RAM AT >MAIN

  /* Uninitialised .bss in AXI SRAM */
  . = ALIGN(4);
  .bss (NOLOAD) :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(SORT_BY_ALIGNMENT(.bss*))
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  . = ALIGN(4);
  .sram2 (NOLOAD) :
  {
    _ssram2 = .;
    __sram2_start__ = _ssram2;
    *(.sram2)
    *(SORT_BY_ALIGNMENT(.sram2*))
    . = ALIGN(4);
    _esram2 = .;
    __sram2_end__ = _esram2;
  } >RAM

  /* FAST_RAM (DTCM) — initialised, stored in MAIN (OCTOSPI) */
  _sfastram_idata = LOADADDR(.fastram_data);
  .fastram_data :
  {
    . = ALIGN(4);
    _sfastram_data = .;
    *(.fastram_data)
    *(.fastram_data*)
    . = ALIGN(4);
    _efastram_data = .;
  } >FASTRAM AT >MAIN

  . = ALIGN(4);
  .fastram_bss (NOLOAD) :
  {
    _sfastram_bss = .;
    __fastram_bss_start__ = _sfastram_bss;
    *(.fastram_bss)
    *(SORT_BY_ALIGNMENT(.fastram_bss*))
    . = ALIGN(4);
    _efastram_bss = .;
    __fastram_bss_end__ = _efastram_bss;
  } >FASTRAM

  /* DMA-safe buffers in AXI SRAM (Write-Through MPU, cache coherent) */
  _sdmaram_idata = LOADADDR(.dmaram_data);

  . = ALIGN(32);
  .dmaram_data :
  {
    PROVIDE(dmaram_start = .);
    _sdmaram = .;
    _dmaram_start__ = _sdmaram;
    _sdmaram_data = .;
    *(.dmaram_data)
    *(.dmaram_data*)
    . = ALIGN(32);
    _edmaram_data = .;
  } >RAM AT >MAIN

  . = ALIGN(32);
  .dmaram_bss (NOLOAD) :
  {
    _sdmaram_bss = .;
    __dmaram_bss_start__ = _sdmaram_bss;
    *(.dmaram_bss)
    *(SORT_BY_ALIGNMENT(.dmaram_bss*))
    . = ALIGN(32);
    _edmaram_bss = .;
    __dmaram_bss_end__ = _edmaram_bss;
  } >RAM

  . = ALIGN(32);
  .DMA_RAM (NOLOAD) :
  {
    KEEP(*(.DMA_RAM))
    PROVIDE(dmaram_end = .);
    _edmaram = .;
    _dmaram_end__ = _edmaram;
  } >RAM

  .DMA_RW_D2 (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmarw_start = .);
    _sdmarw = .;
    _dmarw_start__ = _sdmarw;
    KEEP(*(.DMA_RW))
    PROVIDE(dmarw_end = .);
    _edmarw = .;
    _dmarw_end__ = _edmarw;
  } >D2_RAM

  .DMA_RW_AXI (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmarwaxi_start = .);
    _sdmarwaxi = .;
    _dmarwaxi_start__ = _sdmarwaxi;
    KEEP(*(.DMA_RW_AXI))
    PROVIDE(dmarwaxi_end = .);
    _edmarwaxi = .;
    _dmarwaxi_end__ = _edmarwaxi;
  } >RAM

  .persistent_data (NOLOAD) :
  {
    __persistent_data_start__ = .;
    *(.persistent_data)
    . = ALIGN(4);
    __persistent_data_end__ = .;
  } >RAM

  /* Stack/heap check */
  _heap_stack_end = ORIGIN(STACKRAM) + LENGTH(STACKRAM) - 8;
  _heap_stack_begin = _heap_stack_end - _Min_Stack_Size - _Min_Heap_Size;
  . = _heap_stack_begin;
  ._user_heap_stack :
  {
    . = ALIGN(4);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(4);
  } >STACKRAM = 0xa5

  .memory_b1_text :
  {
    *(.mb1text) *(.mb1text*) *(.mb1rodata) *(.mb1rodata*)
  } >MEMORY_B1

  /DISCARD/ : { libc.a ( * ) libm.a ( * ) libgcc.a ( * ) }
  .ARM.attributes 0 : { *(.ARM.attributes) }

  /* EXST hash block — MD5 hash appended by firmware-patcher tool */
  .exst_hash :
  {
    . = ORIGIN(EXST_HASH);
    BYTE(0x00); /* block format */
    BYTE(0x00); /* checksum method: 0x00 = MD5 */
    BYTE(0x00); /* reserved */
    BYTE(0x00); /* reserved */
    FILL(0x00000000);
    . = ORIGIN(EXST_HASH) + LENGTH(EXST_HASH) - 16;
    __md5_hash_address__ = .;
    LONG(0x00000000);
    LONG(0x00000000);
    LONG(0x00000000);
    LONG(0x00000000);
    . = ORIGIN(EXST_HASH) + LENGTH(EXST_HASH);
    __firmware_end__ = .;
  } >EXST_HASH
}
