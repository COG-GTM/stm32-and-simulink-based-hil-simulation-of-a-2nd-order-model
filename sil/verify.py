#!/usr/bin/env python3
"""Verify Embedded Coder generated C (host SIL run) against the Simulink model.

Oracle: parameters from SystemParameters.m/SampleTime.m (evaluated by Octave)
and block parameters read straight out of the .slx files. Writes plots, an
HTML report and results.json; exits 1 if any check fails.
"""
import argparse
import base64
import json
import re
import sys
import zipfile
from pathlib import Path

import control
import matplotlib
import numpy as np
from scipy import signal

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
SLX = ROOT / "Simulink"
PWM_VMAX = 3.33  # VOLT_REF in both main.c files

TOL = {
    "open_loop_max_err_V": 1e-3,
    "open_loop_outport_transient_V": 5e-3,  # ode4 stage-4 predictor bias, ~2.2 mV
    "open_loop_outport_steady_V": 1e-4,
    "closed_loop_max_err_V": 5e-3,
    "rise_time_s": 0.05,
    "settling_time_s": 0.05,
    "overshoot_pct": 0.5,
    "gain_rel": 1e-9,
}


def slx_params(model):
    """Return {block_name: {param: value}} from a .slx archive."""
    with zipfile.ZipFile(SLX / f"{model}.slx") as z:
        xml = z.read("simulink/systems/system_root.xml").decode()
    blocks = {}
    for m in re.finditer(r'<Block BlockType="(\w+)" Name="([^"]+)".*?</Block>', xml, re.S):
        params = dict(re.findall(r'<P Name="(\w+)"[^>]*>([^<]*)</P>', m.group(0)))
        params["BlockType"] = m.group(1)
        blocks[m.group(2)] = params
    return blocks


def slx_block(model, block_type):
    return next(b for b in slx_params(model).values() if b["BlockType"] == block_type)


def c_constants(path):
    return [abs(float(x)) for x in re.findall(r"(-?\d+\.\d+(?:[eE]-?\d+)?)\s*\*", Path(path).read_text())]


def step_metrics(t, y, y0, y1):
    """10-90 % rise, 2 % settling, overshoot for a step from y0 to y1."""
    d = y1 - y0
    n = (y - y0) / d
    rise = t[np.argmax(n >= 0.9)] - t[np.argmax(n >= 0.1)]
    outside = np.where(np.abs(n - 1.0) > 0.02)[0]
    settle = t[outside[-1] + 1] - t[0] if len(outside) and outside[-1] + 1 < len(t) else t[0] - t[0]
    return {"rise_time_s": float(rise), "settling_time_s": float(settle),
            "overshoot_pct": float(max(0.0, (n.max() - 1.0) * 100))}


def mil_closed_loop(num, den, ts, kp, ki, r):
    """Model-in-the-loop reference: exact ZOH plant + trapezoidal PI, float64."""
    a, b, c, _, _ = signal.cont2discrete(signal.tf2ss([num], den), ts, method="zoh")
    x = np.zeros((2, 1))
    integ_state = 0.0
    y, u = np.zeros(len(r)), np.zeros(len(r))
    for k, rk in enumerate(r):
        y[k] = (c @ x).item()
        e = rk - y[k]
        integ = integ_state + ts / 2 * ki * e
        u[k] = kp * e + integ
        integ_state = integ + ts / 2 * ki * e
        x = a @ x + b * u[k]
    return y, u


def dark():
    plt.style.use("dark_background")
    plt.rcParams.update({"font.size": 15, "axes.titlesize": 19, "axes.titleweight": "bold",
                         "axes.grid": True, "grid.alpha": 0.25, "figure.facecolor": "#0d1117",
                         "axes.facecolor": "#0d1117", "savefig.facecolor": "#0d1117"})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="out")
    ap.add_argument("--plant-c", required=True)
    ap.add_argument("--ctrl-c", required=True)
    ap.add_argument("--ref-c", required=True)
    args = ap.parse_args()
    out = Path(args.out)

    p = json.loads((out / "params.json").read_text())
    num, den, ts = p["num"], p["den"], p["sample_time"]
    ctrl = slx_block("ControllerModel", "Reference")
    pulse = slx_block("ReferenceInputModel", "DiscretePulseGenerator")
    const = slx_block("ReferenceInputModel", "Constant")
    plant_blk = slx_block("PlantModel", "TransferFcn")
    kp, ki = float(ctrl["P"]), float(ctrl["I"])

    checks = []

    def check(name, value, limit, ok):
        checks.append({"check": name, "value": value, "limit": limit, "pass": bool(ok)})

    # 1. Traceability: constants in generated C == model / parameter scripts.
    pc, cc = c_constants(args.plant_c), c_constants(args.ctrl_c)
    rc = [float(x) for x in re.findall(r"\d+\.\d+", Path(args.ref_c).read_text())]
    rel = lambda a, b: abs(a - b) / abs(b)  # noqa: E731
    check("Plant TF uses SystemParameters.m (slx)", f"{plant_blk['Numerator']} / {plant_blk['Denominator']}",
          "[num] / [den]", plant_blk["Numerator"] == "[num]" and plant_blk["Denominator"] == "[den]")
    for label, want in [("plant num = 1/(R1R2C1C2)", num), ("plant den[1]", den[1]), ("plant den[2]", den[2])]:
        err = min(rel(v, want) for v in pc) if pc else 1.0  # A-matrix entries are negated
        check(f"C {label} = {want:g}", f"{err:.1e}", TOL["gain_rel"], err <= TOL["gain_rel"])
    for label, want, consts in [("Kp", kp, cc), ("Ki", ki, cc)]:
        err = min(rel(v, want) for v in consts)
        check(f"C PI {label} = {want:.6g} (slx)", f"{err:.1e}", TOL["gain_rel"], err <= TOL["gain_rel"])
    for label, want in [("pulse amplitude", float(pulse["Amplitude"])), ("constant", float(const["Value"]))]:
        err = min(rel(v, want) for v in rc)
        check(f"C reference {label} = {want:g} (slx)", f"{err:.1e}", TOL["gain_rel"], err <= TOL["gain_rel"])
    err = min(rel(v, ts / 2) for v in cc)
    check(f"C trapezoidal Ts/2 = {ts / 2:g} ({ctrl.get('IntegratorMethod')})", f"{err:.1e}",
          TOL["gain_rel"], err <= TOL["gain_rel"])

    # 2. Open loop: generated plant vs analytic G(s) step response.
    ol = np.genfromtxt(out / "open_loop.csv", delimiter=",", names=True)
    t_ol = ol["t"]
    _, y_an = control.step_response(control.tf([num], den), T=t_ol)
    ol_err = float(np.max(np.abs(ol["y_state"] - y_an)))
    ol_err_out = float(np.max(np.abs(ol["y"] - y_an)))
    steady = t_ol >= 4.0
    ol_err_ss = float(np.max(np.abs(ol["y"][steady] - y_an[steady])))
    check("Open-loop max |SIL - analytic| (state)", f"{ol_err:.2e} V", TOL["open_loop_max_err_V"],
          ol_err <= TOL["open_loop_max_err_V"])
    check("Open-loop max |outport y - analytic| (transient)", f"{ol_err_out:.2e} V",
          TOL["open_loop_outport_transient_V"], ol_err_out <= TOL["open_loop_outport_transient_V"])
    check("Open-loop |outport y - analytic| (steady, t >= 4 s)", f"{ol_err_ss:.2e} V",
          TOL["open_loop_outport_steady_V"], ol_err_ss <= TOL["open_loop_outport_steady_V"])
    m_an = step_metrics(t_ol, y_an, 0.0, 1.0)
    m_ol = step_metrics(t_ol, ol["y_state"], 0.0, 1.0)

    # 3. Closed loop: SIL (generated ref + PI + plant) vs MIL reference.
    cl = np.genfromtxt(out / "closed_loop.csv", delimiter=",", names=True)
    t, r = cl["t"], cl["r"]
    width, period = int(pulse["PulseWidth"]), int(pulse["Period"])
    k = np.arange(len(t))
    r_model = float(const["Value"]) - float(pulse["Amplitude"]) * ((k % period) < width)
    check("Reference r[k] matches slx pulse/constant", f"{np.max(np.abs(r - r_model)):.1e}", 1e-6,
          np.max(np.abs(r - r_model)) <= 1e-6)
    y_mil, u_mil = mil_closed_loop(num, den, ts, kp, ki, r_model)
    cl_err = float(np.max(np.abs(cl["y"] - y_mil)))
    check("Closed-loop max |SIL - MIL|", f"{cl_err:.2e} V", TOL["closed_loop_max_err_V"],
          cl_err <= TOL["closed_loop_max_err_V"])
    seg = slice(0, width)
    m_sil = step_metrics(t[seg], cl["y"][seg], 0.0, r_model[0])
    m_mil = step_metrics(t[seg], y_mil[seg], 0.0, r_model[0])
    for key in ("rise_time_s", "settling_time_s", "overshoot_pct"):
        d = abs(m_sil[key] - m_mil[key])
        unit = "%" if key.endswith("pct") else " s"
        check(f"Closed-loop {key.rsplit('_', 1)[0].replace('_', ' ')}: SIL {m_sil[key]:.3g}{unit} vs MIL "
              f"{m_mil[key]:.3g}{unit}", f"{d:.3g}{unit}", TOL[key], d <= TOL[key])
    u = cl["u"]
    check("Actuator u within PWM range [0, 3.33] V", f"[{u.min():.3f}, {u.max():.3f}]", "[0, 3.33]",
          u.min() >= 0 and u.max() <= PWM_VMAX)

    n_pass = sum(c["pass"] for c in checks)
    verdict = "PASS" if n_pass == len(checks) else "FAIL"
    results = {"verdict": verdict, "passed": n_pass, "total": len(checks), "checks": checks,
               "params": p, "slx": {"Kp": kp, "Ki": ki, "Ts": ts},
               "open_loop": {"analytic": m_an, "sil": m_ol, "max_err_V": ol_err,
                             "max_err_outport_V": ol_err_out,
                             "steady_err_outport_V": ol_err_ss},
               "closed_loop": {"sil": m_sil, "mil": m_mil, "max_err_V": cl_err}}

    # ---- plots -----------------------------------------------------------
    dark()
    wn, zeta = np.sqrt(den[2]), den[1] / (2 * np.sqrt(den[2]))

    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(13, 8), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    tf_fine = np.linspace(0, t_ol[-1], 2000)
    _, y_fine = control.step_response(control.tf([num], den), T=tf_fine)
    ax.plot(tf_fine, y_fine, color="#58a6ff", lw=4, alpha=0.6, label="Analytic G(s) from SystemParameters.m")
    ax.plot(t_ol, ol["y_state"], "o", color="#f78166", ms=5, label="Generated PlantModel.c (gcc, ode4 @ 50 ms)")
    ax.axvline(m_an["settling_time_s"], color="#3fb950", ls="--")
    ax.annotate(f"settles (2%) in {m_ol['settling_time_s']:.2f} s\novershoot {m_ol['overshoot_pct']:.2f} %\n"
                f"rise {m_ol['rise_time_s']:.2f} s", (m_an["settling_time_s"], 0.55),
                xytext=(m_an["settling_time_s"] + 0.4, 0.45), color="#3fb950", fontsize=15)
    ax.set_title(f"Open loop: G(s) = {num:g} / (s² + {den[1]:g}s + {den[2]:g})   ωn={wn:g} rad/s, ζ={zeta:.2f}")
    ax.set_ylabel("y [V]")
    ax.legend(loc="lower right")
    ax.set_xlim(0, 4)
    ax2.plot(t_ol, (ol["y_state"] - y_an) * 1e6, color="#d2a8ff", lw=2)
    ax2.set_ylabel("error [µV]")
    ax2.set_xlabel("t [s]")
    ax2.set_title(f"max |code - model| = {ol_err * 1e6:.1f} µV", fontsize=15)
    fig.tight_layout()
    fig.savefig(out / "01_open_loop_model_vs_code.png", dpi=110)
    plt.close(fig)

    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(13, 8), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    ax.step(t, r_model, where="post", color="#8b949e", lw=2, label="r  (ReferenceInputModel.slx)")
    ax.plot(t, y_mil, color="#58a6ff", lw=4, alpha=0.6, label="MIL: model (ZOH plant + Tustin PI)")
    ax.plot(t, cl["y"], color="#f78166", lw=1.8, ls="--", label="SIL: generated Ref + PI + Plant C")
    ax.plot(t, u, color="#e3b341", lw=1.2, alpha=0.7, label="u  (PWM command)")
    ax.annotate(f"SIL rise {m_sil['rise_time_s']:.2f} s | settle {m_sil['settling_time_s']:.2f} s | "
                f"OS {m_sil['overshoot_pct']:.1f} %\nMIL rise {m_mil['rise_time_s']:.2f} s | settle "
                f"{m_mil['settling_time_s']:.2f} s | OS {m_mil['overshoot_pct']:.1f} %",
                (0.2, 1.62), color="#3fb950", fontsize=14)
    ax.set_ylim(0, 1.9)
    ax.set_title(f"Closed loop SIL vs MIL — PI Kp={kp:.4f}, Ki={ki:.4f}, Ts={ts * 1000:g} ms")
    ax.set_ylabel("[V]")
    ax.legend(loc="lower right", fontsize=12)
    ax2.plot(t, (cl["y"] - y_mil) * 1e3, color="#d2a8ff", lw=2)
    ax2.set_ylabel("err [mV]")
    ax2.set_xlabel("t [s]")
    ax2.set_title(f"max |SIL - MIL| = {cl_err * 1e3:.2f} mV", fontsize=15)
    fig.tight_layout()
    fig.savefig(out / "02_closed_loop_sil_vs_mil.png", dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(14, 0.62 * len(checks) + 1.8))
    ax.axis("off")
    color = "#3fb950" if verdict == "PASS" else "#f85149"
    ax.set_title(f"SIL regression gate: {n_pass}/{len(checks)} {verdict}", color=color, fontsize=24)
    for i, c in enumerate(checks):
        y0 = 1 - (i + 0.5) / len(checks)
        ax.text(0.0, y0, "PASS" if c["pass"] else "FAIL", color="#3fb950" if c["pass"] else "#f85149",
                fontsize=15, weight="bold", va="center", transform=ax.transAxes)
        ax.text(0.08, y0, c["check"], fontsize=14, va="center", transform=ax.transAxes)
        ax.text(0.80, y0, str(c["value"]), fontsize=14, va="center", color="#8b949e", transform=ax.transAxes)
    fig.tight_layout()
    fig.savefig(out / "03_sil_gate.png", dpi=110)
    plt.close(fig)

    (out / "results.json").write_text(json.dumps(results, indent=2))
    imgs = "".join(
        f'<img src="data:image/png;base64,{base64.b64encode((out / n).read_bytes()).decode()}">'
        for n in ("03_sil_gate.png", "01_open_loop_model_vs_code.png", "02_closed_loop_sil_vs_mil.png"))
    rows = "".join(f"<tr class={'ok' if c['pass'] else 'bad'}><td>{'PASS' if c['pass'] else 'FAIL'}</td>"
                   f"<td>{c['check']}</td><td>{c['value']}</td><td>{c['limit']}</td></tr>" for c in checks)
    (out / "sil_report.html").write_text(
        "<html><head><meta charset=utf-8><title>SIL report</title><style>body{background:#0d1117;color:#e6edf3;"
        "font:16px sans-serif;margin:24px}img{max-width:100%;margin:8px 0}td{padding:4px 12px}"
        ".ok td:first-child{color:#3fb950}.bad td:first-child{color:#f85149}</style></head><body>"
        f"<h1 style='color:{color}'>Embedded Coder SIL: {n_pass}/{len(checks)} {verdict}</h1>"
        f"<table>{rows}</table>{imgs}</body></html>")

    for c in checks:
        print(f"[{'PASS' if c['pass'] else 'FAIL'}] {c['check']}: {c['value']} (limit {c['limit']})")
    print(f"SIL gate: {n_pass}/{len(checks)} {verdict}  ->  {out / 'sil_report.html'}")
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
