# -*- coding: utf-8 -*-
import re
import os
import argparse

def log_to_csv(input_file, output_file, acc_scale=1.0, gyro_scale=1.0):
    pattern = r'my_gsensor:\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)'

    lines = []
    with open(input_file, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()

    samples = []
    sample_idx = 0

    for line in lines:
        line = line.strip()

        match = re.search(pattern, line)
        if match:
            ax = float(match.group(1)) * acc_scale
            ay = float(match.group(2)) * acc_scale
            az = float(match.group(3)) * acc_scale
            gx = float(match.group(4)) * gyro_scale
            gy = float(match.group(5)) * gyro_scale
            gz = float(match.group(6)) * gyro_scale

            timestamp = sample_idx * 0.02

            samples.append((timestamp, ax, ay, az, gx, gy, gz))
            sample_idx += 1

    with open(output_file, 'w') as f:
        f.write("timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n")
        for s in samples:
            f.write(f"{s[0]:.3f},{s[1]:.6f},{s[2]:.6f},{s[3]:.6f},{s[4]:.6f},{s[5]:.6f},{s[6]:.6f}\n")

    print(f"Converted {len(samples)} samples")
    print(f"Input:  {input_file}")
    print(f"Output: {output_file}")
    print(f"Duration: {len(samples)*0.02:.1f} seconds")

    if acc_scale != 1.0 or gyro_scale != 1.0:
        print(f"Scale factors: acc x {acc_scale:.4f}, gyro x {gyro_scale:.4f}")

    if samples:
        print(f"\nFirst sample:")
        print(f"  acc = ({samples[0][1]:.4f}, {samples[0][2]:.4f}, {samples[0][3]:.4f}) m/s^2")
        print(f"  gyro = ({samples[0][4]:.4f}, {samples[0][5]:.4f}, {samples[0][6]:.4f}) rad/s")
        acc_mag = (samples[0][1]**2 + samples[0][2]**2 + samples[0][3]**2)**0.5
        print(f"  |acc| = {acc_mag:.4f} m/s^2")

        if acc_mag < 1.0:
            print("\n  WARNING: |acc| < 1 m/s^2")
            print("  Your data is probably NOT in physical units!")
            print("  Try adding: --acc-scale 10.4")
            print("  Example: python log_to_csv.py log.txt out.csv --acc-scale 10.4")

def main():
    parser = argparse.ArgumentParser(description='Convert RTT IMU log to CSV')
    parser.add_argument('input_log', help='Input log file')
    parser.add_argument('output_csv', nargs='?', help='Output CSV file (optional)')
    parser.add_argument('--acc-scale', type=float, default=1.0,
                        help='Acceleration scale factor')
    parser.add_argument('--gyro-scale', type=float, default=1.0,
                        help='Gyroscope scale factor')

    args = parser.parse_args()

    input_file = args.input_log

    if args.output_csv:
        output_file = args.output_csv
    else:
        base = os.path.splitext(input_file)[0]
        output_file = base + ".csv"

    if not os.path.exists(input_file):
        print(f"Error: File not found: {input_file}")
        return

    log_to_csv(input_file, output_file, args.acc_scale, args.gyro_scale)

if __name__ == "__main__":
    main()
