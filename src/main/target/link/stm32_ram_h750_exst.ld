/*
*****************************************************************************
**
**  File        : stm32_ram_h750_exst.ld
**
**  Abstract    : Linker script for STM32H750xB EXST (External Storage) targets.
**                Firmware is loaded by bootloader into CODE_RAM (AXI SRAM).
**                Bootloader copies binary from external storage to 0x24010000
**                and jumps to Reset_Handler there.
**
**                512K AXI-RAM mapped onto AXI bus on D1 domain
**                128K SRAM1 mapped on D2 domain
**                128K SRAM2 mapped on D2 domain
**                 32K SRAM3 mapped on D2 domain
**                 64K SRAM4 mapped on D3 domain
**                 64K ITCM
**                128K DTCM
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/*
0x00000000 to 0x0000FFFF   64K ITCM
0x20000000 to 0x2001FFFF  128K DTCM, main RAM / stack
0x24000000 to 0x2400FFFF   64K AXI SRAM — scratch RAM (before CODE_RAM)
0x24010000 to 0x2407EFFF  448K CODE_RAM — firmware loaded here by bootloader
0x2407F000 to 0x2407FFFF   64B EXST_HASH — MD5 hash appended by build tooling
0x30000000 to 0x3003FFFF  256K D2 SRAM (SRAM1 128K + SRAM2 128K)
*/

/* EXST hash size — 64 bytes, placed at end of CODE_RAM */
_exst_hash_size = 64;

/* Specify the memory areas */
MEMORY
{
    ITCM_RAM (rwx)  : ORIGIN = 0x00000000, LENGTH = 64K
    DTCM_RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 128K
    RAM (rwx)       : ORIGIN = 0x24000000, LENGTH = 64K       /* scratch before CODE_RAM */
    CODE_RAM (rx)   : ORIGIN = 0x24010000, LENGTH = 448K - 64 /* bootloader fixed address — do not change */
    EXST_HASH (rx)  : ORIGIN = 0x24010000 + (448K - 64), LENGTH = 64

    D2_RAM (rwx)    : ORIGIN = 0x30000000, LENGTH = 256K      /* SRAM1 + SRAM2 */

    MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K
    QUADSPI (rx)    : ORIGIN = 0x90000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", DTCM_RAM)
REGION_ALIAS("FASTRAM",  DTCM_RAM)
REGION_ALIAS("MAIN",     CODE_RAM)

/* Highest address of the user mode stack */
_estack = ORIGIN(STACKRAM) + LENGTH(STACKRAM) - 8; /* Reserve 2 x 4bytes for info across reset */

_Min_Heap_Size  = 0;
_Min_Stack_Size = 0x800;

/* Define output sections */
SECTIONS
{
  /* ISR vector table — must be first in MAIN */
  .isr_vector :
  {
    . = ALIGN(512);
    PROVIDE (isr_vector_table_base = .);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >MAIN

  /* Code and read-only data */
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

  /* Critical code in ITCM — zero-flash-latency execution */
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

  /* Initialised .data — stored in MAIN, copied to DTCM at startup */
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

  /* Uninitialised .bss in DTCM */
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
  } >DTCM_RAM

  /* sram2 in RAM (AXI scratch) */
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

  /* FAST_RAM (DTCM) — initialised, stored in MAIN */
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
