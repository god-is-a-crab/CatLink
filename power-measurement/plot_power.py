#!/usr/bin/env python3
import sys
import struct
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


def parse_file(path: str) -> list[float]:
    with open(path, "r") as f:
        raw = f.read()

    # Parse all hex tokens
    tokens = [int(b, 16) for b in raw.split()]

    samples = []
    i = 0
    while i + 2 < len(tokens):
        high, low, delim = tokens[i], tokens[i + 1], tokens[i + 2]
        if delim != 0x0A:
            print(f"Warning: expected delimiter 0x0A at token {i+2}, got {delim:#04x}, skipping")
            i += 1
            continue
        value = (high << 8) | low
        power_mw = value * 0.25
        samples.append(power_mw)
        i += 3

    return samples


def plot(samples: list[float], output_path: str | None = None):
    fig, ax = plt.subplots(figsize=(10, 5))

    SAMPLE_INTERVAL_S = 4.7e-3
    times = [i * SAMPLE_INTERVAL_S for i in range(len(samples))]

    ax.plot(times, samples, linewidth=1.2, color="#2196F3")
    ax.fill_between(times, samples, alpha=0.15, color="#2196F3")

    ax.set_title("INA226 Power", fontsize=14)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Power (mW)")
    ax.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.2f mW"))
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.set_xlim(0, max(times[-1] if times else 1, 1))
    ax.set_ylim(bottom=0)

    fig.tight_layout()

    if output_path:
        fig.savefig(output_path, dpi=150)
        print(f"Saved to {output_path}")
    else:
        plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <data_file> [output.png]")
        sys.exit(1)

    data_file = sys.argv[1]
    out_file = sys.argv[2] if len(sys.argv) > 2 else None

    samples = parse_file(data_file)
    print(f"Parsed {len(samples)} samples")
    if samples:
        print(f"  Min: {min(samples):.2f} mW  Max: {max(samples):.2f} mW  Avg: {sum(samples)/len(samples):.2f} mW")

    plot(samples, out_file)
