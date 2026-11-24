/* Renode smoke-only memory map for the STM32H723 (RM0468).
 *
 * H723 has a single 1 MiB flash bank and 128 KiB of DTCM. The top 4 KiB of
 * DTCM is reserved for the smoke result word; the stack grows down from
 * 0x2001F000, so it can never overwrite the marker at 0x2001F800.
 *
 * This file is NOT the flight linker map. It lives only under emulator/renode.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM   : ORIGIN = 0x20000000, LENGTH = 124K
  SMOKE : ORIGIN = 0x2001F000, LENGTH = 4K
}
