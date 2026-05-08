# -*- coding: utf-8 -*-
"""
IMU Bayes Classifier Trainer
Train Bayesian classifier from IMU data, output C code format model parameters

Usage:
  1. Simulate mode:  python train_bayes.py --mode simulate
  2. CSV mode:       python train_bayes.py --mode csv --dir data/
  3. CSV with files: python train_bayes.py --mode csv --dir data/ --files still.csv,land.csv,sea.csv

CSV format (one sample per line, with header):
  timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
  0.000,     0.021, -0.015, 9.808, 0.001,  -0.002, 0.001
  0.020,     0.019, -0.013, 9.812, 0.000,  -0.001, 0.002
  ...

Output:
  - Console: model parameters + accuracy test
  - File: model_params.h (C header, copy to embedded project)
"""

import argparse
import os
import sys
import math
import random
from collections import defaultdict

SAMPLE_RATE = 25.00
WINDOW_SIZE = 250
FEATURE_DIM = 8
PI = 3.14159265358979323846

MODE_NAMES = ["STILL", "LAND", "SEA"]
FEATURE_NAMES = [
    "acc_std",
    "acc_linear_rms",
    "gyro_std",
    "acc_dominant_freq",
    "acc_low_freq_ratio",
    "acc_high_freq_ratio",
    "acc_periodicity",
    "gyro_periodicity",
]


def vec3_mag(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def arr_mean(data):
    return sum(data) / len(data) if data else 0.0


def arr_stddev(data):
    if not data:
        return 0.0
    m = arr_mean(data)
    s = sum((x - m) ** 2 for x in data)
    return math.sqrt(s / len(data))


def rfft_magnitude(signal):
    n = len(signal)
    if n <= 0 or (n & (n - 1)) != 0:
        fft_n = 1
        while fft_n < n:
            fft_n <<= 1
        padded = list(signal) + [0.0] * (fft_n - n)
        signal = padded
        n = fft_n

    re = list(signal)
    im = [0.0] * n

    bits = 0
    tmp = n
    while tmp > 1:
        bits += 1
        tmp >>= 1

    for i in range(n):
        rev = 0
        for b in range(bits):
            if i & (1 << b):
                rev |= (1 << (bits - 1 - b))
        if i < rev:
            re[i], re[rev] = re[rev], re[i]
            im[i], im[rev] = im[rev], im[i]

    length = 2
    while length <= n:
        angle = -2.0 * PI / length
        wre = math.cos(angle)
        wim = math.sin(angle)
        for i in range(0, n, length):
            cre, cim = 1.0, 0.0
            for j in range(length // 2):
                ure = re[i + j]
                uim = im[i + j]
                vre = cre * re[i + j + length // 2] - cim * im[i + j + length // 2]
                vim = cre * im[i + j + length // 2] + cim * re[i + j + length // 2]
                re[i + j] = ure + vre
                im[i + j] = uim + vim
                re[i + j + length // 2] = ure - vre
                im[i + j + length // 2] = uim - vim
                ncre = cre * wre - cim * wim
                ncim = cre * wim + cim * wre
                cre, cim = ncre, ncim
        length <<= 1

    half = n // 2
    mag = [0.0] * (half + 1)
    for i in range(half + 1):
        mag[i] = math.sqrt(re[i] * re[i] + im[i] * im[i]) / n
    return mag, n


def autocorr_max(signal, max_lag):
    n = len(signal)
    mean = arr_mean(signal)
    var = sum((x - mean) ** 2 for x in signal)
    if var < 1e-12:
        return 0.0
    best = 0.0
    for lag in range(2, min(max_lag + 1, n)):
        s = sum((signal[i] - mean) * (signal[i + lag] - mean) for i in range(n - lag))
        s /= (var * (n - lag))
        if s > best:
            best = s
    return best


def extract_features(readings):
    """
    Extract 8-dim feature vector from IMU readings.
    readings: list of (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
    Returns: list of 8 floats matching C code features_to_vector()
    """
    n = len(readings)

    gx = sum(r[0] for r in readings) / n
    gy = sum(r[1] for r in readings) / n
    gz = sum(r[2] for r in readings) / n

    acc_mag = [vec3_mag(r[0], r[1], r[2]) for r in readings]
    acc_lin_x = [r[0] - gx for r in readings]
    acc_lin_y = [r[1] - gy for r in readings]
    acc_lin_z = [r[2] - gz for r in readings]
    gyro_mag = [vec3_mag(r[3], r[4], r[5]) for r in readings]

    acc_std = arr_stddev(acc_mag)

    lin_rms_sq = sum(vec3_mag(acc_lin_x[i], acc_lin_y[i], acc_lin_z[i]) ** 2 for i in range(n))
    acc_linear_rms = math.sqrt(lin_rms_sq / n)

    gyro_std_val = arr_stddev(gyro_mag)

    acc_m = arr_mean(acc_mag)
    acc_detrend = [acc_mag[i] - acc_m for i in range(n)]

    fft_mag, fft_n = rfft_magnitude(acc_detrend)
    freq_res = SAMPLE_RATE / fft_n

    max_bin = int(5.0 / freq_res)
    if max_bin < 1:
        max_bin = 1
    dom_bin = 1
    dom_val = fft_mag[1]
    for i in range(2, min(max_bin + 1, len(fft_mag))):
        if fft_mag[i] > dom_val:
            dom_val = fft_mag[i]
            dom_bin = i
    acc_dominant_freq = dom_bin * freq_res

    total_e = 0.0
    low_e = 0.0
    mid_e = 0.0
    high_e = 0.0
    for i in range(1, len(fft_mag)):
        e = fft_mag[i] ** 2
        total_e += e
        f = i * freq_res
        if f < 1.0:
            low_e += e
        elif f < 3.0:
            mid_e += e
        else:
            high_e += e
    if total_e > 1e-12:
        acc_low_freq_ratio = low_e / total_e
        acc_high_freq_ratio = high_e / total_e
    else:
        acc_low_freq_ratio = 0.0
        acc_high_freq_ratio = 0.0

    gm = arr_mean(gyro_mag)
    gyro_detrend = [gyro_mag[i] - gm for i in range(n)]

    max_lag = int(5.0 * SAMPLE_RATE)
    if max_lag > n // 2:
        max_lag = n // 2
    acc_periodicity = autocorr_max(acc_detrend, max_lag)
    gyro_periodicity = autocorr_max(gyro_detrend, max_lag)

    return [
        acc_std,
        acc_linear_rms,
        gyro_std_val,
        acc_dominant_freq,
        acc_low_freq_ratio,
        acc_high_freq_ratio,
        acc_periodicity,
        gyro_periodicity,
    ]


def rand_normal(mu, sigma):
    u1 = random.random()
    u2 = random.random()
    while u1 < 1e-10:
        u1 = random.random()
    z = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * PI * u2)
    return mu + sigma * z


def generate_still(n):
    readings = []
    for _ in range(n):
        readings.append((
            rand_normal(0, 0.02),
            rand_normal(0, 0.02),
            rand_normal(9.81, 0.02),
            rand_normal(0, 0.001),
            rand_normal(0, 0.001),
            rand_normal(0, 0.001),
        ))
    return readings


def generate_land(n):
    readings = []
    t = 0.0
    dt = 1.0 / SAMPLE_RATE
    for _ in range(n):
        ax = 0.15 * math.sin(2 * PI * 2.0 * t) + rand_normal(0, 0.08)
        ay = 0.20 * math.sin(2 * PI * 1.5 * t + 0.5) + rand_normal(0, 0.10)
        az = 9.81 + 0.10 * math.sin(2 * PI * 3.0 * t) + rand_normal(0, 0.06)
        gx = 0.010 * math.sin(2 * PI * 1.0 * t) + rand_normal(0, 0.003)
        gy = 0.015 * math.sin(2 * PI * 0.8 * t + 0.3) + rand_normal(0, 0.004)
        gz = 0.005 * math.sin(2 * PI * 2.0 * t + 1.0) + rand_normal(0, 0.002)
        readings.append((ax, ay, az, gx, gy, gz))
        t += dt
    return readings


def generate_sea(n):
    readings = []
    t = 0.0
    dt = 1.0 / SAMPLE_RATE
    for _ in range(n):
        ax = 0.03 * math.sin(2 * PI * 0.08 * t) + 0.02 * math.sin(2 * PI * 0.15 * t + 0.3) + rand_normal(0, 0.02)
        ay = 0.04 * math.sin(2 * PI * 0.12 * t + 0.7) + 0.03 * math.sin(2 * PI * 0.06 * t + 1.1) + rand_normal(0, 0.03)
        az = 9.81 + 0.06 * math.sin(2 * PI * 0.10 * t + 1.2) + 0.04 * math.sin(2 * PI * 0.08 * t + 0.5) + rand_normal(0, 0.03)
        gx = 0.030 * math.sin(2 * PI * 0.08 * t) + 0.020 * math.sin(2 * PI * 0.12 * t + 0.4) + rand_normal(0, 0.005)
        gy = 0.025 * math.sin(2 * PI * 0.12 * t + 0.5) + 0.015 * math.sin(2 * PI * 0.06 * t + 1.2) + rand_normal(0, 0.004)
        gz = 0.015 * math.sin(2 * PI * 0.10 * t + 1.0) + 0.010 * math.sin(2 * PI * 0.08 * t + 0.8) + rand_normal(0, 0.003)
        readings.append((ax, ay, az, gx, gy, gz))
        t += dt
    return readings


def load_csv(filepath):
    """
    Load IMU data from CSV file.
    Expected: timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
    Returns: list of (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
    """
    readings = []
    with open(filepath, "r") as f:
        lines = f.readlines()

    has_timestamp = False
    start = 0
    if lines and lines[0].strip().startswith(("timestamp", "time", "t", "#")):
        start = 1
        has_timestamp = True

    for line in lines[start:]:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.replace(",", " ").split()
        if len(parts) < 6:
            continue
        try:
            if has_timestamp or len(parts) >= 7:
                vals = [float(x) for x in parts[1:7]]
            else:
                vals = [float(x) for x in parts[0:6]]
            readings.append(tuple(vals))
        except ValueError:
            continue

    return readings


def slice_windows(readings, window_size):
    windows = []
    i = 0
    while i + window_size <= len(readings):
        windows.append(readings[i : i + window_size])
        i += window_size
    remainder = len(readings) % window_size
    if remainder > 0:
        print(f"    Warning: {remainder} samples discarded (not a full window)")
    return windows


def train_bayes(feature_vectors, labels):
    """
    Train Bayesian classifier: compute mean/std for each class.
    Returns: (mean[3][8], std[3][8])
    """
    class_features = defaultdict(list)
    for vec, label in zip(feature_vectors, labels):
        class_features[label].append(vec)

    mean = [[0.0] * FEATURE_DIM for _ in range(3)]
    std = [[0.0] * FEATURE_DIM for _ in range(3)]

    for mode in range(3):
        if mode not in class_features or len(class_features[mode]) == 0:
            print(f"  WARNING: No samples for mode {MODE_NAMES[mode]}!")
            continue

        samples = class_features[mode]
        n = len(samples)
        print(f"  {MODE_NAMES[mode]}: {n} windows")

        for d in range(FEATURE_DIM):
            vals = [s[d] for s in samples]
            m = sum(vals) / n
            var = sum((v - m) ** 2 for v in vals) / n
            s = math.sqrt(var)
            if s < 0.001:
                s = 0.001
            mean[mode][d] = m
            std[mode][d] = s

    return mean, std


def classify_bayes(mean, std, feature_vec):
    log_posterior = [0.0, 0.0, 0.0]
    prior = 1.0 / 3.0

    for mode in range(3):
        log_lik = 0.0
        for d in range(FEATURE_DIM):
            diff = (feature_vec[d] - mean[mode][d]) / std[mode][d]
            log_lik += -0.5 * diff * diff - math.log(std[mode][d])
        log_posterior[mode] = log_lik + math.log(prior)

    max_lp = max(log_posterior)
    exp_vals = [math.exp(lp - max_lp) for lp in log_posterior]
    exp_sum = sum(exp_vals)
    probs = [ev / exp_sum for ev in exp_vals]

    best = probs.index(max(probs))
    return best, probs[best]


def generate_c_header(mean, std):
    lines = []
    lines.append("/* ============================================================")
    lines.append(" *  Auto-generated Bayesian model parameters")
    lines.append(" *  Generated by: train_bayes.py")
    lines.append(" *  DO NOT EDIT MANUALLY - regenerate with training script")
    lines.append(" * ============================================================ */")
    lines.append("")
    lines.append("#ifndef MODEL_PARAMS_H")
    lines.append("#define MODEL_PARAMS_H")
    lines.append("")
    lines.append("#define FEATURE_DIM  8")
    lines.append("#define NUM_MODES    3")
    lines.append("")

    lines.append("/* Feature vector order (must match features_to_vector()):")
    for i, name in enumerate(FEATURE_NAMES):
        lines.append(f" *   vec[{i}] = {name}")
    lines.append(" */")
    lines.append("")

    lines.append("static const float s_model_mean[NUM_MODES][FEATURE_DIM] = {")
    for m in range(3):
        vals = ", ".join(f"{mean[m][d]:.6f}f" for d in range(FEATURE_DIM))
        lines.append(f"    {{ {vals} }},  /* {MODE_NAMES[m]} */")
    lines.append("};")
    lines.append("")

    lines.append("static const float s_model_std[NUM_MODES][FEATURE_DIM] = {")
    for m in range(3):
        vals = ", ".join(f"{std[m][d]:.6f}f" for d in range(FEATURE_DIM))
        lines.append(f"    {{ {vals} }},  /* {MODE_NAMES[m]} */")
    lines.append("};")
    lines.append("")

    lines.append("#endif /* MODEL_PARAMS_H */")
    lines.append("")
    return "\n".join(lines)


def print_feature_stats(mean, std):
    print("\n" + "=" * 80)
    print("  Feature Statistics by Transport Mode")
    print("=" * 80)
    print(f"  {'Feature':<25s} | {'Still':>10s} | {'Land':>10s} | {'Sea':>10s} | {'Discrim.':>10s}")
    print("  " + "-" * 75)

    for d in range(FEATURE_DIM):
        vals = [mean[m][d] for m in range(3)]
        max_val = max(vals)
        min_val = min(vals)
        max_idx = vals.index(max_val)
        discrim = (max_val - min_val) / std[max_idx][d] if std[max_idx][d] > 0 else 0
        print(f"  {FEATURE_NAMES[d]:<25s} | {vals[0]:>10.4f} | {vals[1]:>10.4f} | {vals[2]:>10.4f} | {discrim:>10.2f}s")

    print("  " + "-" * 75)
    print(f"  {'(std for each feature):':<25s}")
    print(f"  {'Feature':<25s} | {'Still':>10s} | {'Land':>10s} | {'Sea':>10s} |")
    print("  " + "-" * 75)
    for d in range(FEATURE_DIM):
        vals = [std[m][d] for m in range(3)]
        print(f"  {FEATURE_NAMES[d]:<25s} | {vals[0]:>10.6f} | {vals[1]:>10.6f} | {vals[2]:>10.6f} |")


def run_accuracy_test(mean, std, n_test=100):
    print("\n" + "=" * 80)
    print(f"  Accuracy Test ({n_test} windows/class, simulated data)")
    print("=" * 80)

    random.seed(999)
    generators = [generate_still, generate_land, generate_sea]
    correct = [0, 0, 0]
    total = [0, 0, 0]
    confusion = [[0] * 3 for _ in range(3)]

    for mi in range(3):
        for _ in range(n_test):
            readings = generators[mi](WINDOW_SIZE)
            vec = extract_features(readings)
            pred, conf = classify_bayes(mean, std, vec)
            total[mi] += 1
            if pred == mi:
                correct[mi] += 1
            confusion[mi][pred] += 1

    print(f"\n  {'Mode':<10s} | {'Correct':>8s} | {'Total':>6s} | {'Accuracy':>8s}")
    print("  " + "-" * 45)
    for mi in range(3):
        acc = correct[mi] / total[mi] * 100 if total[mi] > 0 else 0
        print(f"  {MODE_NAMES[mi]:<10s} | {correct[mi]:>8d} | {total[mi]:>6d} | {acc:>7.1f}%")

    total_correct = sum(correct)
    total_all = sum(total)
    print("  " + "-" * 45)
    print(f"  {'Total':<10s} | {total_correct:>8d} | {total_all:>6d} | {total_correct / total_all * 100:>7.1f}%")

    print(f"\n  Confusion Matrix (rows=true, cols=predicted):")
    print(f"  {'':>10s} | {'Still':>8s} | {'Land':>8s} | {'Sea':>8s}")
    print("  " + "-" * 45)
    for mi in range(3):
        row = " | ".join(f"{confusion[mi][j]:>8d}" for j in range(3))
        print(f"  {MODE_NAMES[mi]:>10s} | {row}")


def main():
    parser = argparse.ArgumentParser(description="IMU Bayes Classifier Trainer")
    parser.add_argument("--mode", choices=["simulate", "csv"], default="simulate",
                        help="Data source: simulate or csv")
    parser.add_argument("--dir", default="data",
                        help="Directory containing CSV files")
    parser.add_argument("--files", default=None,
                        help="CSV filenames: still.csv,land.csv,sea.csv")
    parser.add_argument("--samples", type=int, default=100,
                        help="Number of training windows per class (simulate mode)")
    parser.add_argument("--test", type=int, default=100,
                        help="Number of test windows per class")
    parser.add_argument("--output", default="model_params.h",
                        help="Output C header file path")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for reproducibility")
    args = parser.parse_args()

    random.seed(args.seed)

    print("=" * 80)
    print("  IMU Bayesian Classifier Trainer")
    print(f"  Mode: {args.mode}")
    print("=" * 80)

    all_feature_vectors = []
    all_labels = []

    if args.mode == "simulate":
        print(f"\n[1] Generating simulated data ({args.samples} windows/class)...")

        generators = [generate_still, generate_land, generate_sea]
        for mi, gen in enumerate(generators):
            print(f"  Generating {MODE_NAMES[mi]}...", end=" ", flush=True)
            for _ in range(args.samples):
                readings = gen(WINDOW_SIZE)
                vec = extract_features(readings)
                all_feature_vectors.append(vec)
                all_labels.append(mi)
            print("done")

    elif args.mode == "csv":
        if args.files:
            filenames = [f.strip() for f in args.files.split(",")]
        else:
            filenames = ["still.csv", "land.csv", "sea.csv"]

        if len(filenames) != 3:
            print("ERROR: Must provide exactly 3 CSV files (still, land, sea)")
            sys.exit(1)

        print(f"\n[1] Loading data from: {args.dir}/")
        print("  (Missing CSV files will use simulated data)\n")

        for mi, fname in enumerate(filenames):
            filepath = os.path.join(args.dir, fname)
            use_simulated = False

            if os.path.exists(filepath):
                print(f"  {MODE_NAMES[mi]}: {fname}...", end=" ", flush=True)
                readings = load_csv(filepath)
                print(f"{len(readings)} samples", end=" ", flush=True)

                if len(readings) < WINDOW_SIZE:
                    print(f"\n    WARNING: Need at least {WINDOW_SIZE} samples, got {len(readings)}, using SIMULATED")
                    use_simulated = True
            else:
                print(f"  {MODE_NAMES[mi]}: {fname} NOT FOUND, using SIMULATED")
                use_simulated = True

            if use_simulated:
                readings = []
                gen_func = [generate_still, generate_land, generate_sea][mi]
                for _ in range(args.samples):
                    readings.extend(gen_func(WINDOW_SIZE))

            windows = slice_windows(readings, WINDOW_SIZE)
            if not use_simulated:
                print(f"-> {len(windows)} windows")

            for win in windows:
                vec = extract_features(win)
                all_feature_vectors.append(vec)
                all_labels.append(mi)

    print(f"\n[2] Training Bayesian classifier...")
    print(f"  Total samples: {len(all_feature_vectors)}")

    mean, std = train_bayes(all_feature_vectors, all_labels)

    print_feature_stats(mean, std)

    run_accuracy_test(mean, std, args.test)

    print(f"\n[3] Generating C header: {args.output}")
    header_content = generate_c_header(mean, std)
    with open(args.output, "w") as f:
        f.write(header_content)
    print(f"  Written to: {args.output}")

    print("\n[4] C code snippet (copy to imu_classifier_core.c):")
    print("  " + "-" * 60)
    for m in range(3):
        vals = ", ".join(f"{mean[m][d]:.6f}f" for d in range(FEATURE_DIM))
        print(f"  mean[{MODE_NAMES[m]}] = {{ {vals} }}")
    print()
    for m in range(3):
        vals = ", ".join(f"{std[m][d]:.6f}f" for d in range(FEATURE_DIM))
        print(f"  std[{MODE_NAMES[m]}]  = {{ {vals} }}")
    print("  " + "-" * 60)

    print("\nDone!")


if __name__ == "__main__":
    main()
