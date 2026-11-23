/* Renode smoke-only memory map.
 *
 * Mirrors the shipping flash/RAM layout but reserves the top 4 KiB of DTCM
 * for the smoke result word. The linker stack grows down from the end of RAM
 * (0x2001F000), so it can never overwrite the marker at 0x2001F800.
 *
 * This file is NOT the flight linker map. It lives only under emulator/renode.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM   : ORIGIN = 0x20000000, LENGTH = 124K
  SMOKE : ORIGIN = 0x2001F000, LENGTH = 4K
}
