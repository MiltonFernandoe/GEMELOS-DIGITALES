"""
21.11 Digital Twin de un motor electrico (ejemplo integrador)
=============================================================
Junta todo lo anterior en un solo gemelo:

  FISICO:  motor DC (electrico + mecanico + termico) moviendo una bomba
           + MCU (PI de velocidad a 1 kHz, PWM 10 bits)
           + sensores: corriente (ADC 12 bits), encoder 1000 ppr,
             termistor en la CARCASA (lento y no ve el punto caliente)
  GEMELO:  recibe telemetria cada 10 ms y calcula
           - Kalman [i, w, T_carga]          -> par de carga (sensor virtual)
           - RLS sobre  V - Ke*w - L di/dt = R*i  -> R en linea
           - R(T) = R0(1 + alfa(T-25))        -> TEMPERATURA DEL BOBINADO sin sensor
           - eficiencia, energia, indicadores de salud

Genera: 1_tablero.png, 2_temperatura_virtual.png
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.estimacion import RLS, FiltroKalman, discretizar
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.motor import MotorDC, ParamsMotor, matrices_lineales
from gemelo.sensores import Encoder, PuenteH, Sensor

SALIDA = carpeta_resultados(__file__)
DT, T_TOTAL, CADA = 1e-3, 120.0, 10
P_NOM = ParamsMotor()
P_REAL = P_NOM.copia(R0=2.08, Kt=0.051, b=2.4e-5, Tc=0.0024, J=1.05e-4, Rth=6.3)
# El gemelo usa R0, L y Kt IDENTIFICADOS en la puesta en marcha (ver 21.3, error < 1 %);
# la parte mecanica (b, J, Tc) queda con valores de catalogo.
P_GEM = P_NOM.copia(R0=P_REAL.R0, L=P_REAL.L, Kt=P_REAL.Kt)


def referencia(t):
    if t < 5:
        return 110.0 * t / 5
    return 110.0 if t < 35 else (130.0 if t < 70 else (80.0 if t < 95 else 125.0))


def carga(t, w):
    extra = 0.015 if 45 <= t < 80 else 0.0                     # valvula que se cierra a medias
    return 0.20 / 200 ** 2 * w * abs(w) + extra


def main():
    titulo("21.11 Digital Twin de un motor electrico")
    rng = np.random.default_rng(0)
    motor = MotorDC(P_REAL)
    s_i = Sensor(ruido=0.02, rango=(-5, 5), bits=12, semilla=1)
    enc = Encoder(ppr=1000, semilla=2)
    puente = PuenteH(zona_muerta=0.0, bits_pwm=10)
    T_carcasa = P_REAL.T_amb

    # --- gemelo
    def nuevo_kf(R_T):
        A, B = matrices_lineales(P_GEM.copia(R0=R_T), T=25.0)
        F, Bd = discretizar(A, B, DT)
        H = np.array([[1.0, 0, 0], [0, 1.0, 0]])
        return F, Bd, H
    F, Bd, H = nuevo_kf(P_GEM.R0)
    kf = FiltroKalman(F, Bd, H, np.diag([1e-4, 0.05, 2e-6]),
                      np.diag([0.02 ** 2, 0.6]), np.zeros(3), np.eye(3) * 0.1)
    rls = RLS([P_GEM.R0], p0=10.0, lam=0.999)
    i_prev = 0.0
    T_virtual, integ = 25.0, 0.0
    acum = np.zeros(3)

    n = int(T_TOTAL / DT)
    reg = {k: [] for k in ["t", "ref", "w", "w_g", "i", "i_g", "TL", "TL_g", "T", "T_cas", "T_vir",
                           "R", "R_g", "P_e", "P_m"]}
    for k in range(n):
        t = k * DT
        # ===== FISICO (MCU + motor) =====
        w_m = enc.velocidad(motor.theta, DT)
        i_m = s_i.medir(motor.i, t)
        e = referencia(t) - w_m
        integ = np.clip(integ + 0.08 * e * DT, 0, 1)
        duty = np.clip(0.004 * e + integ, 0, 1)
        V = puente.aplicar(duty, DT)
        TL = carga(t, motor.w)
        motor.paso(V, DT, TL)
        T_carcasa += DT * ((P_REAL.T_amb + 0.65 * (motor.T - P_REAL.T_amb)) - T_carcasa) / 25.0
        # ===== GEMELO =====
        kf.predecir(V)
        acum += [V, i_m, w_m]
        if k % CADA == 0 and k > 0:
            kf.corregir([i_m, w_m])
            V_p, i_p, w_p = acum / CADA
            acum[:] = 0
            di_dt = (i_m - i_prev) / (CADA * DT)
            i_prev = i_m
            if abs(i_p) > 0.3 and t > 3:                   # solo con excitacion suficiente
                rls.actualizar([i_p], V_p - P_GEM.Kt * w_p - P_GEM.L * di_dt)
            R_g = rls.theta[0]
            T_obj = 25 + (R_g / P_GEM.R0 - 1) / P_GEM.alfa
            T_virtual += 0.02 * (T_obj - T_virtual)        # suavizado
        if k % 1000 == 0 and t > 3:                          # el gemelo actualiza su modelo con R(T)
            kf.F, kf.B, _ = nuevo_kf(rls.theta[0])
        if k % 20 == 0:
            vals = [t, referencia(t), motor.w, kf.x[1], motor.i, kf.x[0], TL, kf.x[2], motor.T,
                    T_carcasa, T_virtual, motor.R, rls.theta[0], V * motor.i,
                    P_REAL.Kt * motor.i * motor.w]
            for key, v in zip(reg, vals):
                reg[key].append(v)
    r = {k: np.array(v) for k, v in reg.items()}

    ok = r["t"] > 10
    print(f"  Velocidad: error medio gemelo-real          {np.mean(np.abs(r['w_g'] - r['w'])[ok]):6.2f} rad/s")
    print(f"  Par de carga (sin sensor): error medio      {np.mean(np.abs(r['TL_g'] - r['TL'])[ok]) * 1000:6.2f} mN*m")
    print(f"  Temperatura bobinado VIRTUAL: error medio   {np.mean(np.abs(r['T_vir'] - r['T'])[ok]):6.2f} C")
    print(f"  Termistor de CARCASA: error medio            {np.mean(np.abs(r['T_cas'] - r['T'])[ok]):6.2f} C")
    efic = r["P_m"][ok].sum() / r["P_e"][ok].sum() * 100
    print(f"  Eficiencia media: {efic:.1f} %   Energia consumida: {r['P_e'].sum() * 20 * DT:.0f} J")

    # --- 1. tablero
    fig, axes = plt.subplots(3, 2, figsize=(15, 10), sharex=True)
    ax = axes[0, 0]
    ax.plot(r["t"], r["ref"], "k:", label="referencia")
    ax.plot(r["t"], r["w"], color=COL["fisico"], lw=2, label="real")
    ax.plot(r["t"], r["w_g"], "--", color=COL["gemelo"], label="gemelo")
    ax.set_ylabel("Velocidad (rad/s)"); ax.legend(fontsize=8); ax.set_title("Velocidad")
    ax = axes[0, 1]
    ax.plot(r["t"], r["i"], color=COL["fisico"], lw=1.5, label="real")
    ax.plot(r["t"], r["i_g"], "--", color=COL["gemelo"], lw=1, label="gemelo")
    ax.set_ylabel("Corriente (A)"); ax.legend(fontsize=8); ax.set_title("Corriente")
    ax = axes[1, 0]
    ax.plot(r["t"], r["TL"] * 1000, color=COL["fisico"], lw=2, label="real (no se mide)")
    ax.plot(r["t"], r["TL_g"] * 1000, color=COL["gemelo"], lw=1, label="sensor virtual (Kalman)")
    ax.axvspan(45, 80, color=COL["alerta"], alpha=0.1, label="valvula semicerrada")
    ax.set_ylabel("Par de carga (mN*m)"); ax.legend(fontsize=8); ax.set_title("Par de carga: medido sin sensor")
    ax = axes[1, 1]
    ax.plot(r["t"], r["T"], color=COL["fisico"], lw=2, label="bobinado real (inaccesible)")
    ax.plot(r["t"], r["T_vir"], color=COL["gemelo"], label="bobinado VIRTUAL (desde R estimada)")
    ax.plot(r["t"], r["T_cas"], color=COL["modelo"], label="termistor en carcasa")
    ax.axhline(P_NOM.T_max, color=COL["falla"], ls=":", label="limite 130 C")
    ax.set_ylabel("C"); ax.legend(fontsize=8); ax.set_title("Temperatura")
    ax = axes[2, 0]
    ax.plot(r["t"], r["R"], color=COL["fisico"], lw=2, label="R real")
    ax.plot(r["t"], r["R_g"], color=COL["gemelo"], label="R estimada (RLS)")
    ax.set_ylabel("Ohm"); ax.set_xlabel("Tiempo (s)"); ax.legend(fontsize=8)
    ax.set_title("Resistencia del bobinado: sube con la temperatura")
    ax = axes[2, 1]
    ax.plot(r["t"], r["P_e"], color=COL["alerta"], label="potencia electrica")
    ax.plot(r["t"], r["P_m"], color=COL["ok"], label="potencia mecanica")
    ax.set_ylabel("W"); ax.set_xlabel("Tiempo (s)"); ax.legend(fontsize=8)
    ax.set_title(f"Potencia (eficiencia media {efic:.0f} %)")
    fig.suptitle("1. Tablero del gemelo digital del motor: lo que se mide, lo que se estima y lo que se predice",
                 fontsize=12)
    fig.tight_layout(); guardar(fig, SALIDA, "1_tablero.png")

    # --- 2. temperatura virtual vs fisica (detalle)
    fig, ax = plt.subplots(figsize=(11, 4.2))
    ax.plot(r["t"], r["T"], color=COL["fisico"], lw=2.5, label="bobinado real")
    ax.plot(r["t"], r["T_vir"], color=COL["gemelo"], lw=1.5,
            label=f"sensor virtual (error medio {np.mean(np.abs(r['T_vir'] - r['T'])[ok]):.1f} C)")
    ax.plot(r["t"], r["T_cas"], color=COL["modelo"], lw=1.5,
            label=f"termistor carcasa (error medio {np.mean(np.abs(r['T_cas'] - r['T'])[ok]):.1f} C)")
    ax.set_xlabel("Tiempo (s)"); ax.set_ylabel("C"); ax.legend()
    ax.set_title("2. El bobinado es lo que se quema, pero no se puede medir. El gemelo lo deduce de R(T)")
    fig.tight_layout(); guardar(fig, SALIDA, "2_temperatura_virtual.png")
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
