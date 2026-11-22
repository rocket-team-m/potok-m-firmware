/* Minimal linker map for the first revision. TODO(schematic): confirm flash/RAM
 * sizes and the exact STM32H743 variant from the BOM before the first flash. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
