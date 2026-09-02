# ------------------------------------------------------------------
#  check_size.py -- post-build flash/SRAM budget guard
#
#  The ATmega32A has 32 KB of flash and this project already fills most
#  of it, so the thing most likely to break the board is not a bug: it is
#  a build that quietly grows over the urboot bootloader sitting at the
#  top of flash.  Overwrite that and the board can only be recovered with
#  an ISP programmer, which is exactly the situation urboot exists to
#  avoid.  avr-size compares against the whole 32 KB and so never warns.
#
#  This script fails the build instead, against two budgets:
#
#    BOOTLOADER_RESERVE   bytes at the top of flash that belong to the
#                         bootloader and must stay clear.  512 is the
#                         usual urboot footprint for this part; measure
#                         yours and override it if it differs (see the
#                         README, "Flash budget").
#
#    STACK_RESERVE        SRAM left unallocated for the call stack.  The
#                         measured worst-case chain is 182 bytes (see
#                         README, "Stack"); 224 leaves a margin, and the
#                         firmware reports its own high-water mark so the
#                         number can be checked on the device.
#
#  Override either from platformio.ini:
#      board_upload.bootloader_reserve = 1024
# ------------------------------------------------------------------
Import("env")

import os
import subprocess

FLASH_TOTAL = 32768
SRAM_TOTAL  = 2048

DEFAULT_BOOTLOADER_RESERVE = 512
DEFAULT_STACK_RESERVE      = 224


def _int_from_board(env, key, default):
    """board_upload.<key> in platformio.ini, else the default."""
    try:
        value = env.BoardConfig().get("upload", {}).get(key, default)
    except Exception:
        value = default
    try:
        return int(str(value), 0)
    except (TypeError, ValueError):
        return default


def _section_sizes(elf):
    """{name: size} for the allocated sections, via avr-size -A."""
    size_tool = env.subst("$SIZETOOL") or "avr-size"
    try:
        out = subprocess.check_output([size_tool, "-A", elf],
                                      universal_newlines=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        print("check_size: could not run %s (%s)" % (size_tool, exc))
        return None

    sizes = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].startswith("."):
            try:
                sizes[parts[0]] = int(parts[1])
            except ValueError:
                pass
    return sizes


def check(source, target, env):
    elf = str(target[0])
    if not os.path.isfile(elf):
        return

    sizes = _section_sizes(elf)
    if sizes is None:
        return

    text   = sizes.get(".text", 0)
    data   = sizes.get(".data", 0)
    bss    = sizes.get(".bss", 0)
    noinit = sizes.get(".noinit", 0)

    # .data is stored in flash as well as occupying SRAM.
    flash = text + data
    sram  = data + bss + noinit

    boot_reserve  = _int_from_board(env, "bootloader_reserve",
                                    DEFAULT_BOOTLOADER_RESERVE)
    stack_reserve = _int_from_board(env, "stack_reserve",
                                    DEFAULT_STACK_RESERVE)

    flash_budget = FLASH_TOTAL - boot_reserve
    sram_budget  = SRAM_TOTAL - stack_reserve

    print("")
    print("Budget check (see scripts/check_size.py)")
    print("  flash %5d / %5d B  (%5.1f%%)  %d B free below the "
          "bootloader at 0x%04X"
          % (flash, flash_budget, 100.0 * flash / flash_budget,
             flash_budget - flash, flash_budget))
    print("  sram  %5d / %5d B  (%5.1f%%)  %d B free, %d B held back "
          "for the stack"
          % (sram, sram_budget, 100.0 * sram / sram_budget,
             sram_budget - sram, stack_reserve))

    failed = []
    if flash > flash_budget:
        failed.append(
            "flash: %d B of code+data would reach 0x%04X, but the top %d B "
            "(from 0x%04X) are reserved for the urboot bootloader. Over by "
            "%d B." % (flash, flash, boot_reserve, flash_budget,
                       flash - flash_budget))
    if sram > sram_budget:
        failed.append(
            "sram: %d B of static allocation leaves only %d B for the call "
            "stack, and the measured worst case needs 182 B. Over by "
            "%d B." % (sram, SRAM_TOTAL - sram, sram - sram_budget))

    if failed:
        print("")
        for line in failed:
            print("  *** " + line)
        print("")
        print("  Reduce the build (DBG_MODE=0 and DBG_LED_PROBE=0 in "
              "config.h are the two biggest levers), or, if your "
              "bootloader really is smaller, set")
        print("      board_upload.bootloader_reserve = <bytes>")
        print("  in platformio.ini after measuring it.")
        env.Exit(1)
    print("")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", check)
