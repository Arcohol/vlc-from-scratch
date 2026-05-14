#!/usr/bin/env python3
"""Plot processed ADC samples with interactive time-axis pan and zoom."""

from __future__ import annotations

import argparse
import math
import sys
from array import array
from fractions import Fraction
from pathlib import Path
import re
from typing import Iterable, Optional

from split_resample_raw import parse_bit_period_from_name


DEFAULT_SAMPLES_PER_BIT = 16

RATE_ARG_RE = re.compile(
    r"^(?P<value>\d+(?:\.\d+)?)(?P<unit>hz|k|khz|m|mhz)?$",
    re.IGNORECASE,
)
RATE_NAME_RE = re.compile(
    r"_(?P<value>\d+(?:p\d+)?)(?P<unit>Hz|kHz|MHz)_part\d+\.raw$",
    re.IGNORECASE,
)


def positive_float(text: str) -> float:
    value = float(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return value


def parse_rate(value: str) -> float:
    match = RATE_ARG_RE.match(value.strip())
    if not match:
        raise argparse.ArgumentTypeError(
            f"invalid sample rate {value!r}; examples: 400000, 400kHz, 1MHz"
        )

    rate = float(match.group("value"))
    unit = (match.group("unit") or "hz").lower()
    multipliers = {
        "hz": 1,
        "k": 1_000,
        "khz": 1_000,
        "m": 1_000_000,
        "mhz": 1_000_000,
    }
    return rate * multipliers[unit]


def infer_sample_rate(path: Path, samples_per_bit: int) -> float:
    rate_match = RATE_NAME_RE.search(path.name)
    if rate_match:
        value = rate_match.group("value").replace("p", ".")
        return parse_rate(f"{value}{rate_match.group('unit')}")

    bit_period_s = parse_bit_period_from_name(path)
    return float(Fraction(samples_per_bit, 1) / bit_period_s)


def resolve_sample_rate(args: argparse.Namespace) -> Optional[float]:
    if args.sample_axis:
        return None
    if args.sample_interval_us:
        return 1_000_000.0 / args.sample_interval_us
    if args.sample_rate:
        return args.sample_rate
    return infer_sample_rate(args.input, args.samples_per_bit)


def load_samples(path: Path) -> array:
    values = array("i")

    with path.open("r", encoding="ascii") as src:
        for line_number, line in enumerate(src, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                values.append(int(stripped))
            except ValueError as exc:
                raise ValueError(
                    f"{path}:{line_number}: expected one ADC value per line"
                ) from exc

    if not values:
        raise ValueError(f"{path} has no samples")
    return values


class SamplePlot:
    def __init__(
        self,
        values: array,
        title: str,
        sample_rate: Optional[float],
        max_points: int,
        initial_window: Optional[int],
        auto_y: bool,
        line_width: float,
        marker_size: float,
        show_quantized: bool,
    ) -> None:
        import matplotlib.pyplot as plt

        self.plt = plt
        self.values = values
        self.title = title
        self.sample_rate = sample_rate
        self.x_scale = 1.0 / sample_rate if sample_rate else 1.0
        self.max_points = max(100, max_points)
        self.auto_y = auto_y
        self.show_quantized = show_quantized
        self.value_min = min(values)
        self.value_max = max(values)
        self.quantize_threshold = (self.value_min + self.value_max) / 2.0
        self.drag_start_x: Optional[float] = None
        self.drag_start_view: Optional[tuple[float, float]] = None

        self.fig, self.ax = plt.subplots()
        (self.line,) = self.ax.plot(
            [],
            [],
            lw=line_width,
            marker=".",
            markersize=marker_size,
            markeredgewidth=0,
            label="raw",
        )
        (self.quantized_line,) = self.ax.plot(
            [],
            [],
            lw=max(1.0, line_width),
            drawstyle="steps-post",
            color="tab:orange",
            alpha=0.85,
            label="quantized",
        )
        self.ax.set_xlabel("time (s)" if sample_rate else "sample")
        self.ax.set_ylabel("sample value")
        self.ax.grid(True, alpha=0.25)
        self.ax.legend(loc="upper right")
        if not auto_y:
            self.ax.set_ylim(0, 2047)

        if initial_window:
            self.view_start = 0.0
            self.view_end = float(min(len(values), max(2, initial_window)))
        else:
            self.view_start = 0.0
            self.view_end = float(len(values) - 1)

        self.fig.canvas.mpl_connect("scroll_event", self.on_scroll)
        self.fig.canvas.mpl_connect("button_press_event", self.on_button_press)
        self.fig.canvas.mpl_connect("button_release_event", self.on_button_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    def sample_to_x(self, sample: float) -> float:
        return sample * self.x_scale

    def x_to_sample(self, x_value: float) -> float:
        return x_value / self.x_scale

    def clamp_view(self, start: float, end: float) -> tuple[float, float]:
        n = float(len(self.values))
        min_span = 2.0
        max_span = max(min_span, n - 1.0)
        span = min(max(end - start, min_span), max_span)

        start = min(max(start, 0.0), max(0.0, n - span))
        end = start + span
        return start, min(end, n - 1.0)

    def set_view(self, start: float, end: float) -> None:
        self.view_start, self.view_end = self.clamp_view(start, end)
        self.update()

    def zoom_about(self, center_sample: float, factor: float) -> None:
        span = self.view_end - self.view_start
        fraction = (center_sample - self.view_start) / span if span > 0 else 0.5
        new_span = span * factor
        new_start = center_sample - fraction * new_span
        self.set_view(new_start, new_start + new_span)

    def pan_fraction(self, fraction: float) -> None:
        span = self.view_end - self.view_start
        delta = span * fraction
        self.set_view(self.view_start + delta, self.view_end + delta)

    def update(self) -> None:
        i0 = max(0, int(math.floor(self.view_start)))
        i1 = min(len(self.values), int(math.ceil(self.view_end)) + 1)
        if i1 <= i0:
            i1 = min(len(self.values), i0 + 1)

        visible_count = i1 - i0
        stride = max(1, math.ceil(visible_count / self.max_points))
        indices = range(i0, i1, stride)
        x_values = [index * self.x_scale for index in indices]
        visible = [self.values[index] for index in indices]

        self.line.set_data(x_values, visible)
        if self.show_quantized:
            quantized = [
                self.value_max if value >= self.quantize_threshold else self.value_min
                for value in visible
            ]
            self.quantized_line.set_data(x_values, quantized)
        else:
            quantized = []
            self.quantized_line.set_data([], [])
        self.ax.set_xlim(
            self.sample_to_x(self.view_start), self.sample_to_x(self.view_end)
        )

        if self.auto_y and visible:
            y_values = visible + quantized
            y_min = float(min(y_values))
            y_max = float(max(y_values))
            padding = max(1.0, (y_max - y_min) * 0.05)
            self.ax.set_ylim(y_min - padding, y_max + padding)

        self.ax.set_title(
            f"{self.title}  samples={len(self.values)}  "
            f"view={int(self.view_start)}:{int(self.view_end)}  stride={stride}  "
            f"threshold={self.quantize_threshold:g}"
        )
        self.fig.canvas.draw_idle()

    def on_scroll(self, event) -> None:
        if event.inaxes != self.ax or event.xdata is None:
            return
        center = self.x_to_sample(event.xdata)
        factor = 0.80 if event.button == "up" else 1.25
        self.zoom_about(center, factor)

    def on_button_press(self, event) -> None:
        if event.inaxes != self.ax or event.button != 1:
            return
        self.drag_start_x = float(event.x)
        self.drag_start_view = (self.view_start, self.view_end)

    def on_button_release(self, _event) -> None:
        self.drag_start_x = None
        self.drag_start_view = None

    def on_motion(self, event) -> None:
        if (
            self.drag_start_x is None
            or self.drag_start_view is None
            or event.inaxes != self.ax
        ):
            return
        width_pixels = max(1.0, float(self.ax.bbox.width))
        start, end = self.drag_start_view
        samples_per_pixel = (end - start) / width_pixels
        delta_samples = (float(event.x) - self.drag_start_x) * samples_per_pixel
        self.set_view(start - delta_samples, end - delta_samples)

    def on_key(self, event) -> None:
        if event.key in ("+", "="):
            self.zoom_about((self.view_start + self.view_end) / 2.0, 0.80)
        elif event.key in ("-", "_"):
            self.zoom_about((self.view_start + self.view_end) / 2.0, 1.25)
        elif event.key == "left":
            self.pan_fraction(-0.20)
        elif event.key == "right":
            self.pan_fraction(0.20)
        elif event.key == "home":
            self.set_view(0.0, float(len(self.values) - 1))
        elif event.key == "a":
            self.auto_y = not self.auto_y
            if not self.auto_y:
                self.ax.set_ylim(0, 2047)
            self.update()
        elif event.key in ("q", "escape"):
            self.plt.close(self.fig)

    def show(self) -> None:
        self.update()
        print("mouse wheel: zoom X", file=sys.stderr)
        print("left-drag: pan X", file=sys.stderr)
        print(
            "keys: +/- zoom, left/right pan, home reset, a auto-y, q quit",
            file=sys.stderr,
        )
        self.plt.show()


def plot(args: argparse.Namespace) -> int:
    values = load_samples(args.input)
    sample_rate = resolve_sample_rate(args)
    plotter = SamplePlot(
        values=values,
        title=args.title or args.input.name,
        sample_rate=sample_rate,
        max_points=args.max_points,
        initial_window=args.window,
        auto_y=args.auto_y,
        line_width=args.line_width,
        marker_size=args.marker_size,
        show_quantized=args.quantized,
    )
    plotter.show()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="processed values-only .raw file")
    parser.add_argument(
        "--sample-rate",
        type=parse_rate,
        help="processed sample rate in Hz; inferred from filename by default",
    )
    parser.add_argument(
        "--sample-interval-us",
        type=positive_float,
        help="microseconds between adjacent samples",
    )
    parser.add_argument(
        "--sample-axis",
        action="store_true",
        help="show sample number on the X axis instead of time",
    )
    parser.add_argument(
        "--samples-per-bit",
        type=int,
        default=DEFAULT_SAMPLES_PER_BIT,
        help="used only when inferring sample rate from the bit period in the filename",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=25000,
        help="maximum points drawn in the visible view",
    )
    parser.add_argument("--window", type=int, help="initial visible sample count")
    parser.add_argument(
        "--auto-y", action="store_true", help="fit Y axis to visible data"
    )
    parser.add_argument("--line-width", type=float, default=1.0)
    parser.add_argument("--marker-size", type=float, default=8.0)
    parser.add_argument(
        "--no-quantized",
        dest="quantized",
        action="store_false",
        help="hide the midpoint-threshold quantized overlay",
    )
    parser.set_defaults(quantized=True)
    parser.add_argument("--title")
    parser.set_defaults(func=plot)
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if not args.input.is_file():
        parser.error(f"{args.input} is not a file")
    if args.samples_per_bit <= 0:
        parser.error("--samples-per-bit must be greater than zero")
    if args.max_points <= 0:
        parser.error("--max-points must be greater than zero")
    if args.window is not None and args.window <= 0:
        parser.error("--window must be greater than zero")
    if args.marker_size < 0:
        parser.error("--marker-size must be zero or greater")

    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
