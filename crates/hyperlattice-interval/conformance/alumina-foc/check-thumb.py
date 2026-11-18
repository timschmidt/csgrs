#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyelftools==0.32", "unicorn==2.1.4"]
# ///
"""Execute the oracle-checked FOC corpus on a simulated Cortex-M4."""

import argparse
from collections import Counter
from hashlib import sha256
import json
import os
from pathlib import Path
import subprocess
import tempfile

from elftools.elf.elffile import ELFFile
from unicorn import (
    Uc, UcError, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB,
    UC_PROT_EXEC, UC_PROT_READ, UC_PROT_WRITE,
)
from unicorn.arm_const import (
    UC_CPU_ARM_CORTEX_M4, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_PC,
    UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R4, UC_ARM_REG_R5, UC_ARM_REG_R6,
    UC_ARM_REG_R7, UC_ARM_REG_R8, UC_ARM_REG_R9, UC_ARM_REG_R10, UC_ARM_REG_R11,
)


GUEST = Path(__file__).resolve().parent / "wasm"
THUMB = "thumbv7em-none-eabihf"
WASM = "wasm32-unknown-unknown"
FLASH = 0x08000000
FLASH_BYTES = 64 * 1024
RAM = 0x20000000
STACK_BYTES = 8 * 1024
STACK_RESERVE = 4 * 1024
STACK_TOP = RAM + STACK_BYTES
RETURN = FLASH + FLASH_BYTES - 2
ARGUMENTS = (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2)
SAVED = (UC_ARM_REG_R4, UC_ARM_REG_R5, UC_ARM_REG_R6, UC_ARM_REG_R7,
         UC_ARM_REG_R8, UC_ARM_REG_R9, UC_ARM_REG_R10, UC_ARM_REG_R11)
SAVED_VALUES = tuple(0x44440000 + n for n in range(len(SAVED)))
CALLS = {"conformance_probe": 825, "output_word": 3315, "range_probe": 251}


def build(directory, target):
    common = ["--manifest-path", GUEST / "Cargo.toml", "--release", "--locked",
              "--lib", "--target-dir", target]
    subprocess.run(["cargo", "+1.88.0", "build", *common, "--target", WASM], check=True)
    replay = directory / "calls.json"
    subprocess.run(["node", GUEST / "check.mjs",
                    target / WASM / "release/interval_foc_wasm_consumer.wasm", replay],
                   check=True, timeout=60)
    subprocess.run(["cargo", "+1.88.0", "rustc", *common, "--target", THUMB,
                    "--crate-type", "staticlib"], check=True)
    sysroot = Path(subprocess.check_output(["rustc", "+1.88.0", "--print", "sysroot"],
                                          text=True).strip())
    version = subprocess.check_output(["rustc", "+1.88.0", "-vV"], text=True)
    host = next(line.split()[1] for line in version.splitlines() if line.startswith("host: "))
    linker_name = "rust-lld.exe" if os.name == "nt" else "rust-lld"
    linker = sysroot / "lib/rustlib" / host / "bin" / linker_name
    image = directory / "foc.elf"
    subprocess.run([linker, "-flavor", "gnu", "--nmagic", f"-Ttext={FLASH:#x}",
                    "--gc-sections", "--strip-debug", "-e", "conformance_probe",
                    "-u", "range_probe", "-u", "output_word",
                    target / THUMB / "release/libinterval_foc_wasm_consumer.a", "-o", image],
                   check=True)
    return image, json.loads(replay.read_text())


def load_image(image):
    with image.open("rb") as file:
        elf = ELFFile(file)
        if (elf.elfclass != 32 or not elf.little_endian or elf["e_machine"] != "EM_ARM"
                or elf["e_type"] != "ET_EXEC" or not elf["e_flags"] & 0x400):
            raise ValueError("expected a linked 32-bit little-endian hard-float ARM ELF")
        segments = []
        for segment in elf.iter_segments(type="PT_LOAD"):
            start = segment["p_vaddr"]
            end = start + segment["p_memsz"]
            if (start < FLASH or end > FLASH + FLASH_BYTES - 16 * 1024
                    or segment["p_flags"] & 2 or segment["p_filesz"] != segment["p_memsz"]):
                raise ValueError("load segment exceeds the read-only fixture image")
            segments.append((start, segment.data()))
        symbols = elf.get_section_by_name(".symtab")
        entries = {}
        for name in CALLS:
            symbol, = symbols.get_symbol_by_name(name)
            address = symbol["st_value"]
            if symbol["st_info"]["type"] != "STT_FUNC" or not address & 1:
                raise ValueError(f"expected a Thumb function for {name}")
            entries[name] = address
        return segments, entries


def replay_order(segments, entries, calls, pattern):
    cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
    cpu.mem_map(FLASH, FLASH_BYTES, UC_PROT_READ | UC_PROT_EXEC)
    # Leave the reserved lower page unmapped, including during nested calls.
    stack_base = RAM + STACK_RESERVE
    writable = STACK_BYTES - STACK_RESERVE
    cpu.mem_map(stack_base, writable, UC_PROT_READ | UC_PROT_WRITE)
    for address, data in segments:
        cpu.mem_write(address, data)
    cpu.reg_write_batch([*zip(SAVED, SAVED_VALUES), (UC_ARM_REG_SP, STACK_TOP)])
    peaks = []
    sentinel = bytes([pattern]) * writable
    for _ in range(2):  # Cold run, then the same corpus on the live CPU.
        cpu.mem_write(stack_base, sentinel)
        for name, args, value in calls:
            registers = [(reg, arg & 0xffffffff) for reg, arg in zip(ARGUMENTS, args)]
            cpu.reg_write_batch([*registers, (UC_ARM_REG_LR, RETURN | 1)])
            try:
                cpu.emu_start(entries[name], RETURN, count=1_000_000)
            except UcError as error:
                raise RuntimeError(f"Thumb execution fault in {name}{args}") from error
            pc, sp, lo, hi, *saved = cpu.reg_read_batch(
                (UC_ARM_REG_PC, UC_ARM_REG_SP, UC_ARM_REG_R0, UC_ARM_REG_R1, *SAVED))
            if pc != RETURN:
                raise RuntimeError(f"{name}{args} did not return within the 1,000,000-instruction limit")
            if sp != STACK_TOP or tuple(saved) != SAVED_VALUES:
                raise RuntimeError(f"{name}{args} returned with altered caller state")
            width = 32 if name == "range_probe" else 64
            actual = lo if width == 32 else lo | (hi << 32)
            expected = int(value) % (1 << width)
            if actual != expected:
                raise ValueError(f"{name}{args} returned {actual:#x}, expected {expected:#x}")
        memory = cpu.mem_read(stack_base, writable)
        untouched = next((i for i, byte in enumerate(memory) if byte != pattern), writable)
        peaks.append(writable - untouched)
    return peaks


def run(directory, target):
    image, orders = build(directory, target)
    segments, entries = load_image(image)
    if len(orders) != 2:
        raise ValueError("expected forward and reverse WASM replays")
    for order, calls in enumerate(orders):
        indices = range(165) if order == 0 else reversed(range(165))
        digests = [args for name, args, _ in calls if name == "conformance_probe"]
        if (Counter(name for name, _, _ in calls) != CALLS
                or digests != [[index, action] for index in indices for action in range(5)]):
            raise ValueError("incomplete FOC call corpus")
        for pattern in (0x55, 0xaa):
            cold, warm = replay_order(segments, entries, calls, pattern)
            print(f"order {order}, fill {pattern:#04x}, {len(calls)} calls per pass, "
                  f"cold/warm touched stack {cold}/{warm} bytes")
    occupied = max(address + len(data) for address, data in segments) - FLASH
    print(f"Cortex-M4 replay passed, linked load extent {occupied} bytes, "
          f"ELF SHA-256 {sha256(image.read_bytes()).hexdigest()}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="retain the linked ELF and checked calls in a new directory")
    options = parser.parse_args()
    target = Path(os.environ.get("CARGO_TARGET_DIR", GUEST / "target")).resolve()
    if options.output is None:
        with tempfile.TemporaryDirectory() as tmp:
            run(Path(tmp), target)
    else:
        options.output.mkdir(parents=True)
        run(options.output, target)
