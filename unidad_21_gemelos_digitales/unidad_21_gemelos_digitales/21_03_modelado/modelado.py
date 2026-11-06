"""
21.3 Modelado de un dispositivo fisico en Python
================================================
Tres pasos para construir el modelo de un gemelo:
  A) Ecuaciones de primeros principios (motor DC, ver gemelo/motor.py)
  B) Elegir el integrador numerico y el paso de tiempo (Euler vs RK4)
  C) Identificar los parametros reales a partir de un ensayo (minimos cuadrados)

Genera: 1_integradores.png, 2_convergencia.png, 3_ensayo.png, 4_validacion.png
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.motor import MotorDC, ParamsMotor
from gemelo.sensores import Sensor

SALIDA = carpeta_resultados(__file__)


# ---------------------------------------------------------------------
# B) Integradores
# ---------------------------------------------------------------------
def simular_euler(p, V, dt, T):
    m = MotorDC(p)
    i = w = 0.0
    out = []
    for _ in range(int(round(T / dt))):
        di, dw = m._f(i, w, V, 0.0)
        i, w = i + dt * di, w + dt * dw
        out.append(i)
    return np.array(out)


def simular_rk4(p, V, dt, T):
    m = MotorDC(p)
    out = []
    for _ in range(int(round(T / dt))):
        m.paso(V, dt)
        m.T = 25.0                       # sin efecto termico para comparar integradores
        out.append(m.i)
    return np.array(out)


def main():
    titulo("21.3 Modelado de un dispositivo fisico")
    p = ParamsMotor()

    # --- B1. respuesta al escalon con distintos integradores
    T_SIM = 0.03
    ref = simular_rk4(p, 12.0, 1e-5, T_SIM)
    t_ref = np.arange(1, len(ref) + 1) * 1e-5
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(t_ref * 1000, ref, color=COL["fisico"], lw=3, alpha=0.4, label="referencia (RK4, dt=0.01 ms)")
    for dt, est in [(2e-3, "o-"), (4e-3, "s-")]:
        e = simular_euler(p, 12.0, dt, T_SIM); r = simular_rk4(p, 12.0, dt, T_SIM)
        tt = np.arange(1, len(e) + 1) * dt * 1000
        ax.plot(tt, e, est, color=COL["modelo"], ms=4, label=f"Euler dt={dt * 1000:.0f} ms")
        ax.plot(tt, r, est, color=COL["gemelo"], ms=4, label=f"RK4 dt={dt * 1000:.0f} ms")
    ax.set_xlabel("Tiempo (ms)"); ax.set_ylabel("Corriente (A)"); ax.legend(fontsize=8)
    ax.set_title("1. Arranque del motor: con dt grande Euler oscila y se equivoca, RK4 no")
    fig.tight_layout(); guardar(fig, SALIDA, "1_integradores.png")

    # --- B2. orden de convergencia
    dts = np.array([5e-5, 1e-4, 2e-4, 5e-4, 1e-3, 2e-3])
    err_e, err_r = [], []
    for dt in dts:
        n = int(round(T_SIM / dt)); idx = (np.arange(1, n + 1) * dt / 1e-5).round().astype(int) - 1
        err_e.append(np.max(np.abs(simular_euler(p, 12.0, dt, T_SIM) - ref[idx])))
        err_r.append(np.max(np.abs(simular_rk4(p, 12.0, dt, T_SIM) - ref[idx])))
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.loglog(dts * 1000, err_e, "o-", color=COL["modelo"], label="Euler (orden 1)")
    ax.loglog(dts * 1000, err_r, "s-", color=COL["gemelo"], label="RK4 (orden 4)")
    ax.set_xlabel("Paso de tiempo dt (ms)"); ax.set_ylabel("Error maximo de corriente (A)")
    ax.set_title("2. Reducir dt a la mitad: Euler mejora 2x, RK4 mejora 16x"); ax.legend()
    fig.tight_layout(); guardar(fig, SALIDA, "2_convergencia.png")

    # --- C. identificacion de parametros
    real = ParamsMotor(R0=2.15, L=5.6e-3, Kt=0.052, J=1.18e-4, b=2.7e-5, Tc=0.0025)
    rng = np.random.default_rng(0)
    DT, N = 5e-4, 8000                                 # 4 s a 2 kHz
    nivel = rng.choice([4.0, 7.0, 10.0, 12.0], size=N // 200).repeat(200)   # PRBS multinivel
    motor = MotorDC(real)
    s_i, s_w = Sensor(ruido=0.01, semilla=1), Sensor(ruido=0.3, semilla=2)
    I, W, Vv = np.zeros(N), np.zeros(N), nivel
    for k in range(N):
        I[k], W[k] = s_i.medir(motor.i), s_w.medir(motor.w)
        motor.paso(Vv[k], DT)
        motor.T = 25.0
    # Etapa 1 - ecuacion electrica (exacta con ZOH, w actua como entrada):
    #   i[k+1] = a i[k] + cV V[k] + cw w[k],  a = exp(-R dt/L), cV = (1-a)/R, cw = -Kt (1-a)/R
    X = np.column_stack([I[:-1], Vv[:-1], W[:-1]])
    (a, cV, cw), *_ = np.linalg.lstsq(X, I[1:], rcond=None)
    R_est = (1 - a) / cV
    L_est = -R_est * DT / np.log(a)
    Kt_est = -cw / cV
    # Etapa 2 - ecuacion mecanica en forma INTEGRAL (promedia el ruido del sensor):
    #   J (w[k+n] - w[k]) = Kt * int(i) - b * int(w) - Tc * int(signo w)
    n = 40                                             # ventana de 20 ms
    ci = np.concatenate([[0], np.cumsum(I[:-1]) * DT])
    cw_ = np.concatenate([[0], np.cumsum(W[:-1]) * DT])
    cs = np.concatenate([[0], np.cumsum(np.sign(W[:-1])) * DT])
    k = np.arange(0, N - n, n // 4)
    Xm = np.column_stack([ci[k + n] - ci[k], cw_[k + n] - cw_[k], cs[k + n] - cs[k]])
    (g1, g2, g3), *_ = np.linalg.lstsq(Xm, W[k + n] - W[k], rcond=None)
    J_est = Kt_est / g1
    b_est = -g2 * J_est
    Tc_est = -g3 * J_est
    est = dict(R0=R_est, L=L_est, Kt=Kt_est, J=J_est, b=b_est, Tc=Tc_est)
    print(f"\n  {'Parametro':10s}{'Nominal':>12s}{'Real':>12s}{'Identificado':>14s}{'Error':>9s}")
    for k, v in est.items():
        r = getattr(real, k)
        print(f"  {k:10s}{getattr(p, k):12.3g}{r:12.3g}{v:14.3g}{(v - r) / r * 100:8.1f}%")

    t = np.arange(N) * DT
    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(t, Vv, color="k"); axes[0].set_ylabel("V (entrada)")
    axes[1].plot(t, I, color=COL["medido"], lw=0.6); axes[1].set_ylabel("i medida (A)")
    axes[2].plot(t, W, color=COL["medido"], lw=0.6); axes[2].set_ylabel("w medida (rad/s)")
    axes[2].set_xlabel("Tiempo (s)")
    fig.suptitle("3. Ensayo de identificacion: senal multinivel que excita todas las dinamicas")
    fig.tight_layout(); guardar(fig, SALIDA, "3_ensayo.png")

    # --- validacion con otra entrada (nunca vista)
    N2 = 4000
    tv = np.arange(N2) * DT
    Vval = 8 + 4 * np.sin(2 * np.pi * 1.5 * tv) + 2 * np.sign(np.sin(2 * np.pi * 0.4 * tv))
    carga = np.where(tv > 1.0, 0.02, 0.0)
    res = {}
    for nombre, prm in [("real", real), ("nominal (catalogo)", p),
                        ("identificado", real.copia(**est))]:
        m = MotorDC(prm); w = []
        for k in range(N2):
            m.paso(Vval[k], DT, carga[k]); m.T = 25.0; w.append(m.w)
        res[nombre] = np.array(w)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True,
                                   gridspec_kw={"height_ratios": [2, 1]})
    ax1.plot(tv, res["real"], color=COL["fisico"], lw=2.5, label="motor real")
    for n, c in [("nominal (catalogo)", COL["modelo"]), ("identificado", COL["gemelo"])]:
        fit = 100 * (1 - np.linalg.norm(res[n] - res["real"]) /
                     np.linalg.norm(res["real"] - res["real"].mean()))
        ax1.plot(tv, res[n], "--", color=c, label=f"modelo {n}  (ajuste {fit:.1f} %)")
        ax2.plot(tv, res[n] - res["real"], color=c)
        print(f"  Validacion - modelo {n:20s}: ajuste NRMSE = {fit:.1f} %")
    ax1.set_ylabel("Velocidad (rad/s)"); ax1.legend(fontsize=8)
    ax2.set_ylabel("Error (rad/s)"); ax2.set_xlabel("Tiempo (s)  [carga de 0.02 N*m desde t=1 s]")
    fig.suptitle("4. Validacion con una entrada distinta: el modelo identificado reproduce al motor real")
    fig.tight_layout(); guardar(fig, SALIDA, "4_validacion.png")
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
