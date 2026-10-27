"""
Offline evaluation on SYNTHETIC data. This is a PATTERN-RECOVERY test:
"can the model recover rules we planted?"  It is NOT real-world accuracy.
Features are identical to the dashboard's buildFeatureRow():
  sin/cos(hour), sin/cos(dow), causal rolling miss rate (m+0.5)/(n+1), streak.
"""
import numpy as np, pandas as pd, sys, datetime as dt
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import roc_auc_score, average_precision_score, precision_score, recall_score, f1_score

def feats(df):
    X, y, heur = [], [], []
    for _, g in df.groupby("patient_id", sort=False):
        g = g.sort_values("timestamp")
        m = n = streak = 0
        hist = []  # (dow, missed)
        outs = []
        for r in g.itertuples():
            d = dt.datetime.fromisoformat(r.datetime)
            dow = (d.weekday()+1) % 7            # JS getDay(): Sunday=0
            hr = d.hour + d.minute/60
            rate = (m+0.5)/(n+1)
            X.append([np.sin(hr/24*2*np.pi), np.cos(hr/24*2*np.pi),
                      np.sin(dow/7*2*np.pi), np.cos(dow/7*2*np.pi), rate, streak])
            # dashboard heuristic (causal)
            pool = [h for h in hist if h[0] == dow]
            pool = pool if len(pool) >= 3 else hist
            pm = sum(h[1] for h in pool)
            risk = (pm + 0.6)/(len(pool)+3)
            if len(outs) >= 2 and outs[-1] and outs[-2]: risk += 0.15
            heur.append(min(risk, 0.95))
            lab = int(r.status == "missed"); y.append(lab)
            hist.append((dow, lab)); outs.append(lab)
            n += 1
            if lab: m += 1; streak += 1
            else: streak = 0
    return np.array(X), np.array(y), np.array(heur)

def metrics(name, y, p, thr):
    pred = (p >= thr).astype(int)
    return dict(model=name, thr=round(thr, 3),
        AUC=round(roc_auc_score(y, p), 3), PR_AUC=round(average_precision_score(y, p), 3),
        precision=round(precision_score(y, pred, zero_division=0), 3),
        recall=round(recall_score(y, pred, zero_division=0), 3),
        F1=round(f1_score(y, pred, zero_division=0), 3))

def best_thr(y, p):
    ts = np.linspace(0.05, 0.9, 86)
    return ts[int(np.argmax([f1_score(y, (p >= t).astype(int), zero_division=0) for t in ts]))]

def run(df, label):
    df = df.copy()
    df["_ord"] = df.groupby("patient_id").cumcount()
    cut = df.groupby("patient_id")["_ord"].transform("max")*0.8
    tr_mask = df["_ord"] <= cut                      # chronological split per patient
    X, y, heur = feats(df)
    # feats() iterates in groupby order == file order when sorted by patient; align masks
    order = np.concatenate([g.index.values for _, g in df.groupby("patient_id", sort=False)])
    tr = tr_mask.loc[order].values; te = ~tr
    tp = df.loc[order, "true_p"].values
    lr = LogisticRegression(max_iter=1000).fit(X[tr], y[tr])
    p_lr = lr.predict_proba(X[te])[:, 1]
    base = np.full(te.sum(), y[tr].mean())
    out = [metrics("base rate (no features)", y[te], base + 1e-9*np.arange(te.sum()), best_thr(y[tr], np.full(tr.sum(), y[tr].mean()))*0+0.5),
           metrics("Bayesian heuristic", y[te], heur[te], best_thr(y[tr], heur[tr])),
           metrics("Logistic regression (dashboard features)", y[te], p_lr, best_thr(y[tr], lr.predict_proba(X[tr])[:, 1])),
           metrics("Oracle (knows hidden true_p) = ceiling", y[te], tp[te], best_thr(y[tr], tp[tr]))]
    res = pd.DataFrame(out)
    print(f"\n=== {label} | [SYNTHETIC DATA] | train={tr.sum()} test={te.sum()} | test miss rate={y[te].mean():.3f} ===")
    print(res.to_string(index=False))
    return res

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "synthetic_dose_log.csv"
    df = pd.read_csv(path)
    r1 = run(df, "POOLED 20 patients, 80/20 chronological split per patient")
    one = df[df.patient_id == "SYN-01"].reset_index(drop=True)
    r2 = run(one, "SINGLE patient SYN-01 only (180 doses, like one device)")
    with open("results_synthetic.txt", "w") as f:
        f.write("ALL RESULTS ARE ON SYNTHETIC DATA WITH PLANTED RULES - PATTERN RECOVERY, NOT REAL-WORLD ACCURACY\n\n")
        f.write("POOLED\n" + r1.to_string(index=False) + "\n\nSINGLE PATIENT\n" + r2.to_string(index=False) + "\n")
