#!/usr/bin/env python3
"""Resample 1 MHz raw ADC captures and split them into bounded-size chunks."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
import re
from typing import Iterable, Iterator


DEFAULT_INPUT_RATE_HZ = 1_000_000
DEFAULT_SAMPLES_PER_BIT = 16
DEFAULT_MAX_BYTES = "10MB"

PERIOD_RE = re.compile(
    r"^(?P<value>\d+(?:\.\d+)?)(?P<unit>ns|us|µs|μs|ms|s)(?:[_-]|$)",
    re.IGNORECASE,
)
SIZE_RE = re.compile(r"^(?P<value>\d+(?:\.\d+)?)(?P<unit>[kmgt]i?b?|b)?$", re.IGNORECASE)
COUNT_RE = re.compile(r"^(?P<value>\d+(?:\.\d+)?)(?P<unit>[kmgt])?$", re.IGNORECASE)


def decimal_fraction(value: str) -> Fraction:
    if "." not in value:
        return Fraction(int(value), 1)

    whole, frac = value.split(".", 1)
    numerator = int((whole or "0") + frac)
    denominator = 10 ** len(frac)
    return Fraction(numerator, denominator)


def parse_bit_period_from_name(path: Path) -> Fraction:
    match = PERIOD_RE.match(path.name)
    if not match:
        raise ValueError(
            f"{path.name!r} does not start with a bit period like '40us_' or '0.5ms_'"
        )

    value = decimal_fraction(match.group("value"))
    unit = match.group("unit").lower()
    multipliers = {
        "ns": Fraction(1, 1_000_000_000),
        "us": Fraction(1, 1_000_000),
        "µs": Fraction(1, 1_000_000),
        "μs": Fraction(1, 1_000_000),
        "ms": Fraction(1, 1_000),
        "s": Fraction(1, 1),
    }
    return value * multipliers[unit]


def parse_size(value: str) -> int:
    match = SIZE_RE.match(value.strip())
    if not match:
        raise ValueError(f"invalid byte size {value!r}; examples: 10M, 10MB, 10MiB")

    amount = decimal_fraction(match.group("value"))
    unit = (match.group("unit") or "b").lower()
    multipliers = {
        "b": 1,
        "k": 1_000,
        "kb": 1_000,
        "m": 1_000_000,
        "mb": 1_000_000,
        "g": 1_000_000_000,
        "gb": 1_000_000_000,
        "t": 1_000_000_000_000,
        "tb": 1_000_000_000_000,
        "ki": 1024,
        "kib": 1024,
        "mi": 1024**2,
        "mib": 1024**2,
        "gi": 1024**3,
        "gib": 1024**3,
        "ti": 1024**4,
        "tib": 1024**4,
    }
    return int(amount * multipliers[unit])


def parse_count(value: str) -> int:
    match = COUNT_RE.match(value.strip())
    if not match:
        raise ValueError(f"invalid sample count {value!r}; examples: 10000000, 10M")

    amount = decimal_fraction(match.group("value"))
    unit = (match.group("unit") or "").lower()
    multipliers = {
        "": 1,
        "k": 1_000,
        "m": 1_000_000,
        "g": 1_000_000_000,
        "t": 1_000_000_000_000,
    }
    return int(amount * multipliers[unit])


def format_rate(rate_hz: Fraction) -> str:
    if rate_hz.denominator == 1:
        hz = rate_hz.numerator
        if hz % 1_000_000 == 0:
            return f"{hz // 1_000_000}MHz"
        if hz % 1_000 == 0:
            return f"{hz // 1_000}kHz"
        return f"{hz}Hz"

    return f"{float(rate_hz):.6g}Hz".replace(".", "p")


def format_rate_for_log(rate_hz: Fraction) -> str:
    if rate_hz.denominator == 1:
        return f"{rate_hz.numerator} Hz"
    return f"{float(rate_hz):.6f} Hz"


def round_fraction(numerator: int, denominator: int) -> int:
    if numerator >= 0:
        return (numerator + denominator // 2) // denominator
    return -((-numerator + denominator // 2) // denominator)


def read_adc_samples(path: Path) -> Iterator[int]:
    with path.open("rb", buffering=1024 * 1024) as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line:
                continue

            try:
                yield int(line)
            except ValueError as exc:
                raise ValueError(f"{path}:{line_number}: expected an integer ADC sample") from exc


def resample_linear(samples: Iterable[int], step_input_samples: Fraction) -> Iterator[int]:
    iterator = iter(samples)
    try:
        previous = next(iterator)
    except StopIteration:
        return

    step_num = step_input_samples.numerator
    step_den = step_input_samples.denominator
    target_pos_num = 0
    previous_index = 0

    for current_index, current in enumerate(iterator, start=1):
        while True:
            base_index, frac_num = divmod(target_pos_num, step_den)
            if base_index >= current_index:
                break

            delta = current - previous
            value_num = previous * step_den + delta * frac_num
            yield round_fraction(value_num, step_den)

            target_pos_num += step_num

        previous = current
        previous_index = current_index

    base_index, frac_num = divmod(target_pos_num, step_den)
    if base_index == previous_index and frac_num == 0:
        yield previous


@dataclass
class ChunkedWriter:
    output_dir: Path
    output_stem: str
    max_samples: int | None
    max_bytes: int | None
    overwrite: bool

    def __post_init__(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.part_index = 0
        self.current_samples = 0
        self.current_bytes = 0
        self.current_path: Path | None = None
        self.current_handle = None
        self.paths: list[Path] = []

    def __enter__(self) -> "ChunkedWriter":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def close(self) -> None:
        if self.current_handle is not None:
            self.current_handle.close()
            self.current_handle = None

    def _next_path(self) -> Path:
        path = self.output_dir / f"{self.output_stem}_part{self.part_index:04d}.raw"
        self.part_index += 1
        if path.exists() and not self.overwrite:
            raise FileExistsError(f"{path} already exists; pass --overwrite to replace it")
        return path

    def _open_next(self) -> None:
        self.close()
        self.current_path = self._next_path()
        self.current_handle = self.current_path.open("wb")
        self.current_samples = 0
        self.current_bytes = 0
        self.paths.append(self.current_path)
        print(f"  writing {self.current_path}")

    def write_sample(self, sample: int) -> None:
        line = f"{sample}\n".encode("ascii")
        if self.max_bytes is not None and len(line) > self.max_bytes:
            raise ValueError(f"one encoded sample is larger than the chunk limit: {line!r}")

        should_roll = self.current_handle is None
        if self.current_handle is not None:
            if self.max_samples is not None and self.current_samples >= self.max_samples:
                should_roll = True
            if self.max_bytes is not None and self.current_bytes + len(line) > self.max_bytes:
                should_roll = True

        if should_roll:
            self._open_next()

        self.current_handle.write(line)
        self.current_samples += 1
        self.current_bytes += len(line)


def convert_file(
    input_path: Path,
    output_dir: Path,
    input_rate_hz: int,
    samples_per_bit: int,
    max_samples: int | None,
    max_bytes: int | None,
    overwrite: bool,
) -> None:
    bit_period_s = parse_bit_period_from_name(input_path)
    target_rate_hz = Fraction(samples_per_bit, 1) / bit_period_s
    step_input_samples = Fraction(input_rate_hz, 1) / target_rate_hz
    output_stem = f"{input_path.stem}_{format_rate(target_rate_hz)}"

    chunk_limits = []
    if max_samples is not None:
        chunk_limits.append(f"{max_samples} samples")
    if max_bytes is not None:
        chunk_limits.append(f"{max_bytes} bytes")
    limit_text = " and ".join(chunk_limits)

    print(
        f"{input_path}: bit period {float(bit_period_s) * 1_000_000:.6g} us, "
        f"target {format_rate_for_log(target_rate_hz)}, chunks <= {limit_text}"
    )

    sample_count = 0
    with ChunkedWriter(output_dir, output_stem, max_samples, max_bytes, overwrite) as writer:
        for sample in resample_linear(read_adc_samples(input_path), step_input_samples):
            writer.write_sample(sample)
            sample_count += 1

    print(f"  wrote {sample_count} samples across {len(writer.paths)} file(s)")


def collect_inputs(paths: list[str], raw_dir: Path) -> list[Path]:
    if paths:
        inputs = []
        for path in paths:
            input_path = Path(path)
            if not input_path.exists() and not input_path.is_absolute():
                candidate = raw_dir / input_path
                if candidate.exists():
                    input_path = candidate
            inputs.append(input_path)
        return inputs

    return sorted(raw_dir.glob("*.raw"))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Convert newline-delimited 1 MHz ADC raw captures into smaller "
            "resampled chunks. The bit period is read from the first filename "
            "component, e.g. 40us_capture.raw -> 400 kHz at 16 samples/bit."
        )
    )
    parser.add_argument(
        "inputs",
        nargs="*",
        help="Raw files to process. Defaults to every *.raw file in --raw-dir.",
    )
    parser.add_argument("--raw-dir", type=Path, default=Path("data/raw"))
    parser.add_argument("--out-dir", type=Path, default=Path("data/processed"))
    parser.add_argument("--input-rate", type=int, default=DEFAULT_INPUT_RATE_HZ)
    parser.add_argument("--samples-per-bit", type=int, default=DEFAULT_SAMPLES_PER_BIT)
    parser.add_argument(
        "--max-samples",
        default="0",
        help="Optional maximum output samples per chunk. Disabled by default.",
    )
    parser.add_argument(
        "--max-bytes",
        default=DEFAULT_MAX_BYTES,
        help="Maximum encoded bytes per chunk, e.g. 10MB or 10MiB.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace existing output chunks. By default, existing chunks cause an error.",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    max_samples = parse_count(args.max_samples)
    if max_samples < 0:
        parser.error("--max-samples must be zero or greater")

    max_bytes = parse_size(args.max_bytes) if args.max_bytes is not None else None
    if max_bytes is not None and max_bytes <= 0:
        parser.error("--max-bytes must be greater than zero")
    if max_samples == 0 and max_bytes is None:
        parser.error("at least one chunk limit is required; set --max-samples or --max-bytes")

    max_samples_limit = max_samples or None
    if args.input_rate <= 0:
        parser.error("--input-rate must be greater than zero")
    if args.samples_per_bit <= 0:
        parser.error("--samples-per-bit must be greater than zero")

    inputs = collect_inputs(args.inputs, args.raw_dir)
    if not inputs:
        parser.error(f"no raw files found in {args.raw_dir}")

    for input_path in inputs:
        if not input_path.is_file():
            parser.error(f"{input_path} is not a file")
        convert_file(
            input_path=input_path,
            output_dir=args.out_dir,
            input_rate_hz=args.input_rate,
            samples_per_bit=args.samples_per_bit,
            max_samples=max_samples_limit,
            max_bytes=max_bytes,
            overwrite=args.overwrite,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
