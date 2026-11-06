"""
21.9 Prediccion del comportamiento del sistema
==============================================
El gemelo sincronizado se adelanta al futuro: "si seguimos asi, que pasa?"

Caso: motor con bomba. A los 6 minutos llega un pedido que exige subir la
demanda de la bomba un 50 %. El operador pregunta cual de tres planes de
trabajo puede cumplirlo sin sobrecalentar el motor en los proximos 10 min.

  - Prediccion con INCERTIDUMBRE (Monte Carlo: los parametros no se conocen exactos)
  - Probabilidad de superar el limite y tiempo hasta superarlo
  - Validacion: se ejecuta el plan A en el motor "real" y se compara
  - Como crece el error de prediccion con el horizonte

Genera: 1_escenarios.png, 2_validacion.png, 3_horizonte.png
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.motor import MotorDC, ParamsMotor

SALIDA = carpeta_resultados(__file__)
DT = 1.0
P_NOM = ParamsMotor()
P_REAL = P_NOM.copia(Rth=6.4, Cth=10.8, R0=2.06)
K_HIST = 0.30 / 200 ** 2            # carga tipo bomba (par = k * w^2) hasta ahora
K_BOMBA = 0.45 / 200 ** 2           # demanda pedida para los proximos 10 minutos
T_AHORA, HORIZONTE = 360.0, 600.0
N_MC = 400

PLANES = {
    "A: seguir a 12 V": lambda t: 12.0,
    "B: bajar a 10 V": lambda t: 10.0,
    "C: ciclos 12 V / 6 V cada 60 s": lambda t: 12.0 if int(t // 60) % 2 == 0 else 6.0,
}


def simular_vectorizado(T0, plan, t0, horizonte, R0, Rth, Cth, k, T_amb, p=P_NOM):
    """Simula muchos motores a la vez (un elemento por muestra de Monte Carlo)."""
    T = np.array(T0, dtype=float) * np.ones_like(R0)
    salida = [T.copy()]
    for paso in range(int(horizonte / DT)):
        V = plan(t0 + paso * DT)
        R = R0 * (1 + p.alfa * (T - 25.0))
        D = p.b + p.Kt ** 2 / R
        N = np.maximum(p.Kt * V / R - p.Tc, 0)
        w = (-D + np.sqrt(D * D + 4 * k * N)) / (2 * k)
        i = (V - p.Kt * w) / R
        T = T + DT * (R * i ** 2 - (T - T_amb) / Rth) / Cth
        salida.append(T.copy())
    return np.array(salida)                       # (pasos+1, N_MC)


def main():
    titulo("21.9 Prediccion del comportamiento del sistema")
    rng = np.random.default_rng(0)

    # --- historia real hasta "ahora" (12 V constantes)
    real = MotorDC(P_REAL)
    hist_t, hist_T = [], []
    for k in range(int(T_AHORA / DT)):
        real.paso_lento(12.0, DT, k_cuad=K_HIST)
        hist_t.append(k * DT); hist_T.append(real.T)
    T_medida = real.T + rng.normal(0, 0.5)          # el gemelo se sincroniza con la medicion

    # --- incertidumbre de parametros (lo que el gemelo sabe de si mismo)
    mc = dict(R0=rng.normal(P_NOM.R0, 0.03 * P_NOM.R0, N_MC),
              Rth=rng.normal(P_NOM.Rth, 0.08 * P_NOM.Rth, N_MC),
              Cth=rng.normal(P_NOM.Cth, 0.10 * P_NOM.Cth, N_MC),
              k=rng.normal(K_BOMBA, 0.05 * K_BOMBA, N_MC),
              T_amb=rng.normal(25, 1.5, N_MC))
    t_fut = T_AHORA + np.arange(int(HORIZONTE / DT) + 1) * DT

    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8), sharey=True)
    resumen = {}
    for ax, (nombre, plan) in zip(axes, PLANES.items()):
        Tmc = simular_vectorizado(T_medida, plan, T_AHORA, HORIZONTE, **mc)
        p5, p50, p95 = np.percentile(Tmc, [5, 50, 95], axis=1)
        supera = (Tmc > P_NOM.T_max).any(axis=0)
        prob = supera.mean() * 100
        t_cruce = [t_fut[np.argmax(col > P_NOM.T_max)] - T_AHORA for col in Tmc.T if (col > P_NOM.T_max).any()]
        resumen[nombre] = (prob, np.median(t_cruce) if t_cruce else np.nan, p50[-1])
        ax.plot(np.array(hist_t) / 60, hist_T, color=COL["fisico"], lw=2, label="historia real")
        ax.fill_between(t_fut / 60, p5, p95, color=COL["pred"], alpha=0.25, label="90 % de confianza")
        ax.plot(t_fut / 60, p50, color=COL["pred"], lw=2, label="prediccion (mediana)")
        for j in range(15):
            ax.plot(t_fut / 60, Tmc[:, j], color=COL["pred"], lw=0.3, alpha=0.5)
        ax.axhline(P_NOM.T_max, color=COL["falla"], ls="--", label="limite 130 C")
        ax.axvline(T_AHORA / 60, color="gray", ls=":")
        txt = f"P(superar) = {prob:.0f} %"
        if t_cruce:
            txt += f"\nen ~{np.median(t_cruce) / 60:.1f} min"
        ax.text(0.03, 0.97, txt, transform=ax.transAxes, va="top", fontsize=10,
                bbox=dict(fc="white", ec=COL["falla"] if prob > 10 else COL["ok"]))
        ax.set_title(nombre); ax.set_xlabel("Tiempo (min)")
    axes[0].set_ylabel("Temperatura del bobinado (C)"); axes[0].legend(fontsize=7, loc="lower right")
    fig.suptitle("1. Que pasa si...? El gemelo evalua tres planes con 400 simulaciones cada uno")
    fig.tight_layout(); guardar(fig, SALIDA, "1_escenarios.png")
    print(f"  {'Plan':32s}{'P(>130 C)':>10s}{'cruza en':>12s}{'T final':>10s}")
    for n, (pr, tc, tf) in resumen.items():
        print(f"  {n:32s}{pr:9.0f}%{(f'{tc / 60:.1f} min' if np.isfinite(tc) else '-'):>12s}{tf:9.1f} C")

    # --- 2. validacion: se ejecuta el plan A en el motor real
    real_val = MotorDC(P_REAL); real_val.T = real.T
    T_real_fut = [real_val.T]
    for k in range(int(HORIZONTE / DT)):
        real_val.paso_lento(PLANES["A: seguir a 12 V"](T_AHORA + k * DT), DT, k_cuad=K_BOMBA)
        T_real_fut.append(real_val.T)
    Tmc = simular_vectorizado(T_medida, PLANES["A: seguir a 12 V"], T_AHORA, HORIZONTE, **mc)
    p5, p50, p95 = np.percentile(Tmc, [5, 50, 95], axis=1)
    dentro = np.mean((np.array(T_real_fut) >= p5) & (np.array(T_real_fut) <= p95)) * 100
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.fill_between(t_fut / 60, p5, p95, color=COL["pred"], alpha=0.25, label="prediccion 90 %")
    ax.plot(t_fut / 60, p50, color=COL["pred"], lw=2, label="prediccion (mediana)")
    ax.plot(t_fut / 60, T_real_fut, color=COL["fisico"], lw=2, label="lo que paso en realidad")
    ax.axhline(P_NOM.T_max, color=COL["falla"], ls="--")
    ax.set_xlabel("Tiempo (min)"); ax.set_ylabel("C"); ax.legend()
    ax.set_title(f"2. Validacion del plan A: la realidad cae dentro de la banda el {dentro:.0f} % del tiempo")
    fig.tight_layout(); guardar(fig, SALIDA, "2_validacion.png")
    print(f"\n  Validacion: la temperatura real quedo dentro de la banda del 90 % el {dentro:.0f} % del tiempo")

    # --- 3. error vs horizonte (muchas predicciones desde distintos instantes)
    # la demanda de la bomba cambia sin aviso en el motor real: la prediccion se degrada con el horizonte
    k_real = lambda t: K_BOMBA * (1 + 0.25 * np.sin(2 * np.pi * t / 900))
    V_real = lambda t: 12.0 if int(t // 240) % 3 else 9.0
    motor = MotorDC(P_REAL)
    traza = []
    for k in range(int(3600 / DT)):
        motor.paso_lento(V_real(k * DT), DT, k_cuad=k_real(k * DT)); traza.append(motor.T)
    traza = np.array(traza)
    horizontes = [10, 30, 60, 120, 300, 600]
    errores = {h: [] for h in horizontes}
    for t0 in range(300, 3000, 60):
        pred = simular_vectorizado(traza[t0], V_real, t0, max(horizontes),
                                   np.array([P_NOM.R0]), np.array([P_NOM.Rth]), np.array([P_NOM.Cth]),
                                   np.array([k_real(t0)]), np.array([25.0]))[:, 0]
        for h in horizontes:
            errores[h].append(abs(pred[h] - traza[t0 + h]))
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(horizontes, [np.mean(errores[h]) for h in horizontes], "o-", color=COL["pred"], label="error medio")
    ax.plot(horizontes, [np.percentile(errores[h], 95) for h in horizontes], "s--", color=COL["falla"],
            label="percentil 95")
    ax.set_xscale("log"); ax.set_xlabel("Horizonte de prediccion (s)"); ax.set_ylabel("Error (C)")
    ax.set_title("3. Cuanto mas lejos se predice, mas error: hay que re-sincronizar y re-predecir")
    ax.legend(); fig.tight_layout(); guardar(fig, SALIDA, "3_horizonte.png")
    for h in horizontes:
        print(f"  horizonte {h:4d} s -> error medio {np.mean(errores[h]):5.2f} C")
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
