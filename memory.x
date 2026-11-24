/* STM32H723ZG linker map for the first revision.
 *
 * Exact source (not invented): ST datasheet DS12939, family STM32H723/733,
 * as encoded in Embassy's stm32-metapac v21.0.0,
 *   src/chips/stm32h723zg/metadata.rs
 *   ITCM    0x00000000  64K
 *   FLASH   0x08000000  1024K (BANK_1, 128K erase sectors)
 *   DTCM    0x20000000  128K
 *   AXISRAM 0x24000000  320K
 *   SRAM2   0x30000000  16K
 *   SRAM3   0x30004000  16K
 *   SRAM4   0x38000000  16K
 * The datasheet PDF was not reachable offline; the cited generated chip
 * metadata is the authority used for every number above.
 *
 * NOT ASSIGNED: the BOM has not fixed the exact H723 package/flash variant.
 * `zg` (1 MB FLASH, LQFP144) is an assumption; `ze` is the same line with
 * 512 KB FLASH. Do not flash until BU01 is signed off.
 * Only DTCM is wired as `RAM` below. ITCM/AXISRAM/SRAM2/3/4 are NOT ASSIGNED
 * to the linker map until the package and the firmware memory plan are
 * confirmed from the schematic. No size here is guessed.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
