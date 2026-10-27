"""
SYNTHETIC dose-log generator (source=synthetic).
Every dose has a KNOWN outcome drawn from a KNOWN hidden probability (true_p),
so there is no confusion between "absent record" and "missed".
Planted rules (all documented, nothing hidden from the reader):
  - baseline miss rate per patient (random effect)
  - evening doses missed more than morning
  - weekends missed more than weekdays
  - miss streak: after a miss, next dose more likely missed
  - slow drift: adherence fatigue grows over the 90 days
"""
import numpy as np, pandas as pd, argparse, datetime as dt

def sigmoid(x): return 1/(1+np.exp(-x))

def generate(n_patients=20, days=90, seed=42):
    rng = np.random.default_rng(seed)
    start = dt.datetime(2026, 1, 5)          # a Monday, fixed for reproducibility
    rows = []
    for pid in range(1, n_patients+1):
        base    = rng.normal(-2.8, 0.4)      # ~6% baseline miss
        evening = rng.normal(0.9, 0.3)
        weekend = rng.normal(0.7, 0.3)
        streak  = rng.normal(0.6, 0.2)
        drift   = rng.normal(0.004, 0.002)   # logit increase per day
        prev_streak = 0
        for d in range(days):
            for hour in (8, 20):
                ts = start + dt.timedelta(days=d, hours=hour)
                is_wkend = ts.weekday() >= 5
                logit = (base + evening*(hour == 20) + weekend*is_wkend
                         + streak*min(prev_streak, 3) + drift*d)
                p = float(sigmoid(logit))
                missed = int(rng.random() < p)
                rows.append(dict(patient_id=f"SYN-{pid:02d}", timestamp=int(ts.timestamp()),
                                 datetime=ts.isoformat(), status="missed" if missed else "confirmed",
                                 true_p=round(p, 4), source="synthetic"))
                prev_streak = prev_streak+1 if missed else 0
    return pd.DataFrame(rows)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--patients", type=int, default=20)
    ap.add_argument("--days", type=int, default=90)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default="synthetic_dose_log.csv")
    a = ap.parse_args()
    df = generate(a.patients, a.days, a.seed)
    df.to_csv(a.out, index=False)
    print(f"[SYNTHETIC] wrote {len(df)} doses, {a.patients} patients, miss rate {(df.status=='missed').mean():.3f} -> {a.out}")
