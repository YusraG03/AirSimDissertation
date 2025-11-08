# Plot path & collisions from a day4_random_*.csv
# Run: python scripts/plot_random_baseline.py --file data/logs/day4_random_YYYYMMDD_HHMMSS.csv

import argparse, os, pandas as pd, matplotlib.pyplot as plt

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", required=True)
    args = ap.parse_args()

    df = pd.read_csv(args.file)
    fig1 = plt.figure()
    plt.plot(df["x"], df["y"], linewidth=1.5)
    plt.xlabel("x (m)"); plt.ylabel("y (m)"); plt.title("Random baseline path (xy)")
    fig2 = plt.figure()
    plt.plot(df["t"], df["z"])
    plt.xlabel("t (s)"); plt.ylabel("z (m)"); plt.title("Altitude over time")

    # collisions timeline
    fig3 = plt.figure()
    plt.step(df["t"], df["collided"], where="post")
    plt.xlabel("t (s)"); plt.ylabel("collision flag"); plt.title("Collisions vs time")

    out_dir = "docs/figures"; os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(args.file))[0]
    fig1.savefig(f"{out_dir}/{base}_path.png", dpi=160, bbox_inches="tight")
    fig2.savefig(f"{out_dir}/{base}_alt.png", dpi=160, bbox_inches="tight")
    fig3.savefig(f"{out_dir}/{base}_collisions.png", dpi=160, bbox_inches="tight")
    print("Saved figures to", out_dir)

if __name__ == "__main__":
    main()
