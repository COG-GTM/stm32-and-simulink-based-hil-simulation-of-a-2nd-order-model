#!/usr/bin/env python3
"""Compare the Embedded Coder generated PlantModel (run on host) against the
analytic 2nd-order Butterworth transfer function from Simulink/SystemParameters.m.

            1/(R1 R2 C1 C2)                    25
 G(s) = -------------------------------- = ----------------
        s^2 + (1/R1+1/R2)/C1 s + 1/(R1R2C1C2)   s^2 + 7.5 s + 25

Writes PNGs + a JSON summary to demo/out/ and exits non-zero if the generated
code deviates from the model beyond tolerance.
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

OUT = Path(__file__).resolve().parent / "out"
TS = 0.05
TOL_RK4 = 1e-3     # generated ode4 state vs analytic (float32 + RK4 truncation, h*w up to ~1)
TOL_API = 5e-3     # PlantModel_Y.y as read on target (stage-4 predictor)

# --- plant from SystemParameters.m ------------------------------------------
C1, C2, R1, R2 = 200e-9, 100e-9, 2e6, 1e6
num = [1 / (R1 * R2 * C1 * C2)]
den = [1, ((1 / R1) + (1 / R2)) / C1, 1 / (R1 * R2 * C1 * C2)]
G = signal.TransferFunction(num, den)
wn = np.sqrt(den[2])
zeta = den[1] / (2 * wn)

# --- controller from ControllerModel.c (discrete PI, forward Euler) ---------
KP, KI = 0.21484784713862, 1.91054639755173


def analytic(t_in, u):
    """Exact continuous response of G sampled at t_in + TS (ZOH input, fine grid)."""
    dt = TS / 200
    tf = np.arange(0, t_in[-1] + TS + dt, dt)
    uf = u[np.minimum((tf // TS).astype(int), len(u) - 1)]
    _, yf, _ = signal.lsim(G, uf, tf)
    return np.interp(t_in + TS, tf, yf)


def load(name):
    d = np.genfromtxt(OUT / f"{name}.csv", delimiter=",", names=True)
    return d


def compare(name, ax_top, ax_bot, title):
    d = load(name)
    t, u, y_api, y_rk4 = d["t"], d["u"], d["y"], d["y_rk4"]
    y_ref = analytic(t, u)
    e_rk4 = y_rk4 - y_ref
    e_api = y_api - y_ref

    ax_top.plot(t, u, color="0.6", lw=1, label="u  (input)")
    ax_top.plot(t + TS, y_ref, "k-", lw=2.2, label="analytic  scipy.signal.lsim")
    ax_top.plot(t + TS, y_rk4, "--", color="tab:red", lw=1.4,
                label="Embedded Coder C  (ode4 state, host gcc)")
    ax_top.set_title(title)
    ax_top.set_ylabel("V")
    ax_top.grid(alpha=.3)
    ax_top.legend(loc="lower right", fontsize=8)

    ax_bot.plot(t + TS, e_rk4, color="tab:red", lw=1.2,
                label=f"code - model   max |e| = {np.abs(e_rk4).max():.2e}")
    ax_bot.plot(t + TS, e_api, color="tab:orange", lw=0.8, alpha=.8,
                label=f"PlantModel_Y.y (as read on MCU)   max |e| = {np.abs(e_api).max():.2e}")
    ax_bot.axhline(0, color="k", lw=.5)
    ax_bot.set_ylabel("error [V]")
    ax_bot.set_xlabel("t [s]")
    ax_bot.grid(alpha=.3)
    ax_bot.legend(loc="upper right", fontsize=8)
    return {"max_err_rk4": float(np.abs(e_rk4).max()),
            "max_err_api": float(np.abs(e_api).max()),
            "rms_err_rk4": float(np.sqrt(np.mean(e_rk4 ** 2)))}


def main():
    OUT.mkdir(exist_ok=True)
    plt.rcParams.update({"figure.dpi": 110})
    summary = {"plant": {"num": num, "den": den, "wn_rad_s": float(wn), "zeta": float(zeta),
                         "Ts": TS, "solver": "ode4"}}

    # 1) step + chirp overlay/error ------------------------------------------
    fig, ax = plt.subplots(2, 2, figsize=(14, 7.5), sharex="col",
                           gridspec_kw={"height_ratios": [3, 1.3]})
    summary["step"] = compare("step", ax[0, 0], ax[1, 0],
                              "Unit step  –  generated C vs analytic G(s)=25/(s²+7.5s+25)")
    summary["chirp"] = compare("chirp", ax[0, 1], ax[1, 1],
                               "Chirp 0.05→3 Hz  –  generated C vs analytic")
    fig.suptitle("Simulink PlantModel.slx → Embedded Coder → PlantModel.c (compiled with gcc on host)",
                 fontsize=13, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT / "01_step_chirp_vs_analytic.png")

    # 2) frequency response: Bode of G vs amplitude ratio recovered from chirp -
    d = load("chirp")
    t, u, y = d["t"], d["u"], d["y_rk4"]
    f_inst = 0.05 + (3.0 - 0.05) / 16.0 * t             # chirp instantaneous freq
    w, mag, phase = signal.bode(G, np.logspace(-1.5, 1.6, 400))
    fig, ax = plt.subplots(1, 2, figsize=(14, 4.8))
    ax[0].semilogx(w / (2 * np.pi), mag, "k", lw=2, label="analytic |G(jω)|")
    # envelope of generated-code chirp response, per instantaneous-frequency bin
    bins = np.linspace(0.3, 3.0, 23)          # below 0.3 Hz a bin holds < 1 cycle
    fc, env = [], []
    for lo, hi in zip(bins[:-1], bins[1:]):
        m = (f_inst >= lo) & (f_inst < hi)
        if m.sum() > 3:
            fc.append((lo + hi) / 2)
            env.append(20 * np.log10(np.abs(y[m]).max() / np.abs(u[m]).max()))
    ax[0].plot(fc, env, "o", color="tab:red", label="generated C – chirp envelope")
    ax[0].axvline(wn / (2 * np.pi), color="tab:blue", ls=":", label=f"f_n = {wn/(2*np.pi):.3f} Hz")
    ax[0].set_xlabel("Hz"); ax[0].set_ylabel("dB"); ax[0].grid(alpha=.3, which="both")
    ax[0].set_title("Magnitude response"); ax[0].legend(fontsize=8)
    ax[1].plot(t, u, color="0.6", lw=.8, label="chirp input")
    ax[1].plot(t + TS, y, color="tab:red", lw=1.2, label="generated C output")
    ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("V"); ax[1].grid(alpha=.3)
    ax[1].set_title("Chirp sweep through the plant"); ax[1].legend(fontsize=8)
    fig.suptitle(f"2nd-order low-pass: ω_n = {wn:.2f} rad/s, ζ = {zeta:.3f}", fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT / "02_frequency_response.png")

    # 3) closed loop: generated PI controller + generated plant vs scipy -----
    d = load("closedloop")
    t, r, u, y = d["t"], d["r"], d["u"], d["y"]
    # reference closed loop: same discrete PI on exact continuous plant (ZOH)
    Gd = signal.cont2discrete((num, den), TS, method="zoh")
    b, a = np.squeeze(Gd[0]), Gd[1]
    y_ref = np.zeros_like(t); u_ref = np.zeros_like(t)
    dstate = 0.0; yz = [0.0] * 3; uz = [0.0] * 3
    for k in range(len(t)):
        e = r[k] - (y_ref[k - 1] if k else 0.0)
        # DiscreteIntegrator (trapezoidal) exactly as emitted in ControllerModel.c
        integ = dstate + 0.5 * TS * KI * e
        uk = np.clip(KP * e + integ, 0.0, 3.3)
        dstate = integ + 0.5 * TS * KI * e
        uz = [uk] + uz[:2]
        yk = b[0] * uz[0] + b[1] * uz[1] + b[2] * uz[2] - a[1] * yz[0] - a[2] * yz[1]
        yz = [yk] + yz[:2]
        y_ref[k], u_ref[k] = yk, uk
    fig, ax = plt.subplots(2, 1, figsize=(14, 7), sharex=True,
                           gridspec_kw={"height_ratios": [3, 1.5]})
    ax[0].step(t, r, where="post", color="k", lw=1.5, label="r  (ReferenceInputModel.c)")
    ax[0].plot(t, y, color="tab:red", lw=1.8, label="y  (PlantModel.c, driven by ControllerModel.c)")
    ax[0].plot(t, y_ref, "--", color="tab:blue", lw=1.1, label="y  scipy reference (ZOH plant + same PI, 1-sample feedback delay)")
    ax[0].set_ylabel("V"); ax[0].grid(alpha=.3); ax[0].legend(fontsize=8, loc="lower right")
    ax[0].set_title("Closed loop on host: three Embedded Coder models linked together (r → PI → plant)")
    ax[1].step(t, u, where="post", color="tab:green", lw=1.2, label="u  PI output → PWM duty (0–3.3 V)")
    ax[1].set_ylabel("V"); ax[1].set_xlabel("t [s]"); ax[1].grid(alpha=.3); ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "03_closed_loop.png")
    summary["closedloop"] = {"max_err_vs_scipy": float(np.abs(y - y_ref).max()),
                             "steady_state_err": float(abs(y[-2] - r[-2]))}

    # --- verdict --------------------------------------------------------------
    ok = (summary["step"]["max_err_rk4"] < TOL_RK4 and summary["chirp"]["max_err_rk4"] < TOL_RK4
          and summary["step"]["max_err_api"] < TOL_API and summary["chirp"]["max_err_api"] < TOL_API)
    summary["verdict"] = "PASS" if ok else "FAIL"
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2))

    print(f"\n  Plant  G(s) = {num[0]:.1f} / (s^2 + {den[1]:.2f} s + {den[2]:.1f})"
          f"   wn = {wn:.2f} rad/s  zeta = {zeta:.3f}   Ts = {TS*1000:.0f} ms  ode4")
    print("  ------------------------------------------------------------------")
    for k in ("step", "chirp"):
        s = summary[k]
        print(f"  {k:<6}  max|code-model| = {s['max_err_rk4']:.2e} V   rms = {s['rms_err_rk4']:.2e} V"
              f"   (MCU-read Y: {s['max_err_api']:.2e} V)")
    print(f"  closed  max|y - scipy ref| = {summary['closedloop']['max_err_vs_scipy']:.2e} V"
          f"   steady-state error = {summary['closedloop']['steady_state_err']:.2e} V")
    print(f"  tolerance {TOL_RK4:.0e} V  ->  VERDICT: {summary['verdict']}\n")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
