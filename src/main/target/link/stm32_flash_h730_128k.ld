/*
*****************************************************************************
**
**  File        : stm32_flash_h730_128k.ld
**
**  Abstract    : Linker script for STM32H730xB Device with
**                320K AXI-SRAM mapped onto AXI bus on D1 domain
**                 16K SRAM1 mapped on D2 domain
**                 16K SRAM2 mapped on D2 domain
**                 64K ITCM
**                128K DTCM
**
**  Note        : H730 has only 128K internal flash. No config sector split.
**                Boards requiring external flash storage use EXST linker.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/*
0x00000000 to 0x0000FFFF   64K ITCM
0x20000000 to 0x2001FFFF  128K DTCM
0x24000000 to 0x2404FFFF  320K AXI SRAM, D1 domain, main RAM
0x30000000 to 0x30003FFF   16K SRAM1, D2 domain
0x30004000 to 0x30007FFF   16K SRAM2, D2 domain
0x38000000 to 0x38003FFF   16K SRAM4, D3 domain, unused

0x08000000 to 0x0801FFFF  128K flash (all code + data init)
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)        : ORIGIN = 0x08000000, LENGTH = 128K

    ITCM_RAM (rwx)    : ORIGIN = 0x00000000, LENGTH = 64K
    DTCM_RAM (rwx)    : ORIGIN = 0x20000000, LENGTH = 128K
    RAM (rwx)         : ORIGIN = 0x24000000, LENGTH = 320K

    D2_RAM (rwx)      : ORIGIN = 0x30000000, LENGTH = 32K /* SRAM1 + SRAM2 */

    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K
}

REGION_ALIAS("STACKRAM", DTCM_RAM)
REGION_ALIAS("FASTRAM", DTCM_RAM)

/* Highest address of the user mode stack */
_estack = ORIGIN(STACKRAM) + LENGTH(STACKRAM) - 8;

/* No dedicated config sector on H730 */
__config_start = ORIGIN(FLASH);
__config_end   = ORIGIN(FLASH);

_Min_Heap_Size = 0;
_Min_Stack_Size = 0x800;

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(512);
    PROVIDE (isr_vector_table_base = .);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text) *(.text*) *(.rodata) *(.rodata*)
    *(.glue_7) *(.glue_7t) *(.eh_frame)
    KEEP (*(.init)) KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  tcm_code = LOADADDR(.tcm_code);
  .tcm_code :
  {
    . = ALIGN(4);
    tcm_code_start = .;
    *(.tcm_code) *(.tcm_code*)
    . = ALIGN(4);
    tcm_code_end = .;
  } >ITCM_RAM AT >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH

  .pg_registry :
  {
    PROVIDE_HIDDEN (__pg_registry_start = .);
    KEEP (*(.pg_registry))
    KEEP (*(SORT(.pg_registry.*)))
    PROVIDE_HIDDEN (__pg_registry_end = .);
  } >FLASH

  .pg_resetdata :
  {
    PROVIDE_HIDDEN (__pg_resetdata_start = .);
    KEEP (*(.pg_resetdata))
    PROVIDE_HIDDEN (__pg_resetdata_end = .);
  } >FLASH

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data) *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT >FLASH

  . = ALIGN(4);
  .bss (NOLOAD) :
  {
    _sbss = .; __bss_start__ = _sbss;
    *(.bss) *(SORT_BY_ALIGNMENT(.bss*)) *(COMMON)
    . = ALIGN(4);
    _ebss = .; __bss_end__ = _ebss;
  } >RAM

  . = ALIGN(4);
  .sram2 (NOLOAD) :
  {
    _ssram2 = .; __sram2_start__ = _ssram2;
    *(.sram2) *(SORT_BY_ALIGNMENT(.sram2*))
    . = ALIGN(4);
    _esram2 = .; __sram2_end__ = _esram2;
  } >RAM

  _sfastram_idata = LOADADDR(.fastram_data);
  .fastram_data :
  {
    . = ALIGN(4);
    _sfastram_data = .;
    *(.fastram_data) *(.fastram_data*)
    . = ALIGN(4);
    _efastram_data = .;
  } >FASTRAM AT >FLASH

  . = ALIGN(4);
  .fastram_bss (NOLOAD) :
  {
    _sfastram_bss = .; __fastram_bss_start__ = _sfastram_bss;
    *(.fastram_bss) *(SORT_BY_ALIGNMENT(.fastram_bss*))
    . = ALIGN(4);
    _efastram_bss = .; __fastram_bss_end__ = _efastram_bss;
  } >FASTRAM

  _sdmaram_idata = LOADADDR(.dmaram_data);

  . = ALIGN(32);
  .dmaram_data :
  {
    PROVIDE(dmaram_start = .);
    _sdmaram = .; _dmaram_start__ = _sdmaram; _sdmaram_data = .;
    *(.dmaram_data) *(.dmaram_data*)
    . = ALIGN(32);
    _edmaram_data = .;
  } >RAM AT >FLASH

  . = ALIGN(32);
  .dmaram_bss (NOLOAD) :
  {
    _sdmaram_bss = .; __dmaram_bss_start__ = _sdmaram_bss;
    *(.dmaram_bss) *(SORT_BY_ALIGNMENT(.dmaram_bss*))
    . = ALIGN(32);
    _edmaram_bss = .; __dmaram_bss_end__ = _edmaram_bss;
  } >RAM

  . = ALIGN(32);
  .DMA_RAM (NOLOAD) :
  {
    KEEP(*(.DMA_RAM))
    PROVIDE(dmaram_end = .);
    _edmaram = .; _dmaram_end__ = _edmaram;
  } >RAM

  .DMA_RW_D2 (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmarw_start = .); _sdmarw = .; _dmarw_start__ = _sdmarw;
    KEEP(*(.DMA_RW))
    PROVIDE(dmarw_end = .); _edmarw = .; _dmarw_end__ = _edmarw;
  } >D2_RAM

  .DMA_RW_AXI (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmarwaxi_start = .); _sdmarwaxi = .; _dmarwaxi_start__ = _sdmarwaxi;
    KEEP(*(.DMA_RW_AXI))
    PROVIDE(dmarwaxi_end = .); _edmarwaxi = .; _dmarwaxi_end__ = _edmarwaxi;
  } >RAM

  .persistent_data (NOLOAD) :
  {
    __persistent_data_start__ = .;
    KEEP(*(.persistent_data))
    . = ALIGN(4);
    __persistent_data_end__ = .;
  } >RAM

  _heap_stack_end = ORIGIN(STACKRAM)+LENGTH(STACKRAM) - 8;
  _heap_stack_begin = _heap_stack_end - _Min_Stack_Size - _Min_Heap_Size;
  . = _heap_stack_begin;
  ._user_heap_stack :
  {
    . = ALIGN(4);
    PROVIDE ( end = . ); PROVIDE ( _end = . );
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
}
