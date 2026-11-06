"""
21.13 Digital Twin de un sistema industrial
===========================================
Planta de dos tanques en cascada (bomba -> tanque 1 -> valvula -> tanque 2 -> consumo),
controlada por un PLC/MCU con un PI de nivel. Tres usos del gemelo en la industria:

  A) PUESTA EN MARCHA VIRTUAL: probar 3 sintonias del controlador en el gemelo
     antes de tocar la planta real, elegir la mejor y confirmarla.
  B) DETECCION DE FUGAS: balance de masa del gemelo vs mediciones.
  C) "QUE PASA SI FALLA LA BOMBA?": el gemelo calcula en todo momento cuantos
     minutos de autonomia quedan antes de que el tanque 2 se vacie.

Genera: 1_puesta_en_marcha.png, 2_fuga.png, 3_autonomia.png
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.proceso import ParamsTanques, Tanques
from gemelo.sensores import Sensor

SALIDA = carpeta_resultados(__file__)
DT = 1.0                                                  # ciclo del PLC: 1 s
P_NOM = ParamsTanques()
P_REAL = P_NOM.copia(k_b=0.0115, c12=0.0083, c2=0.0088, A1=3.12)
H_MIN = 0.30


def consumo(t):
    """Apertura de la valvula de consumo: varia durante el turno."""
    return float(np.clip(0.45 + 0.25 * np.sin(2 * np.pi * t / 7200) + (0.2 if 4800 < t < 6000 else 0), 0, 1))


def setpoint(t):
    return 1.0 if t < 2400 else 1.3


class PI:
    def __init__(self, kp, ti):
        self.kp, self.ti, self.I = kp, ti, 0.5

    def __call__(self, e):
        u = self.kp * e + self.I
        if 0 < u < 1:                                       # anti-windup
            self.I += self.kp / self.ti * e * DT
        return float(np.clip(u, 0, 1))


def correr(p, kp, ti, T=7200, semilla=0, fuga_desde=None):
    planta = Tanques(p, h1=0.6, h2=1.0)
    s_h1, s_h2 = Sensor(ruido=0.005, semilla=semilla), Sensor(ruido=0.005, semilla=semilla + 1)
    pi = PI(kp, ti)
    reg = {k: [] for k in ["t", "h1", "h2", "u", "h1_m", "h2_m", "a", "fuga"]}
    u_prev = 0.5
    for k in range(int(T / DT)):
        t = k * DT
        if fuga_desde is not None and t >= fuga_desde:
            planta.fuga = 0.0012
        h1_m, h2_m = s_h1.medir(planta.h1), s_h2.medir(planta.h2)
        u = pi(setpoint(t) - h2_m)
        a = consumo(t)
        planta.paso(u, a, DT)
        for key, v in zip(reg, [t, planta.h1, planta.h2, u, h1_m, h2_m, a,
                                planta.fuga * np.sqrt(planta.h1)]):
            reg[key].append(v)
        u_prev = u
    return {k: np.array(v) for k, v in reg.items()}


def parte_a():
    sintonias = {"agresiva (Kp=15, Ti=15 min)": (15.0, 900), "moderada (Kp=4, Ti=30 min)": (4.0, 1800),
                 "lenta (Kp=1.5, Ti=60 min)": (1.5, 3600)}
    fig, axes = plt.subplots(2, 2, figsize=(14, 7), sharex="col",
                             gridspec_kw={"width_ratios": [2, 1.2]})
    resumen = {}
    for (n, (kp, ti)), c in zip(sintonias.items(), [COL["modelo"], COL["gemelo"], COL["alerta"]]):
        r = correr(P_NOM, kp, ti)                                   # SOLO en el gemelo
        err = np.mean(np.abs(r["h2"] - [setpoint(t) for t in r["t"]]))
        sobre = (r["h2"][2400:4000].max() - 1.3) * 100
        desgaste = np.sum(np.abs(np.diff(r["u"])))
        resumen[n] = (err * 100, sobre, desgaste)
        axes[0, 0].plot(r["t"] / 60, r["h2"], color=c, label=n)
        axes[1, 0].plot(r["t"] / 60, r["u"] * 100, color=c, lw=0.8)
    axes[0, 0].plot(r["t"] / 60, [setpoint(t) for t in r["t"]], "k:", label="consigna")
    axes[0, 0].set_ylabel("Nivel tanque 2 (m)"); axes[0, 0].legend(fontsize=8)
    axes[0, 0].set_title("En el GEMELO: tres sintonias probadas sin riesgo")
    axes[1, 0].set_ylabel("Bomba (%)"); axes[1, 0].set_xlabel("Tiempo (min)")
    # confirmacion en la planta real con la elegida
    kp, ti = sintonias["moderada (Kp=4, Ti=30 min)"]
    g = correr(P_NOM, kp, ti); rr = correr(P_REAL, kp, ti, semilla=5)
    axes[0, 1].plot(g["t"] / 60, g["h2"], color=COL["gemelo"], label="gemelo")
    axes[0, 1].plot(rr["t"] / 60, rr["h2"], color=COL["fisico"], label="planta real")
    axes[0, 1].plot(g["t"] / 60, [setpoint(t) for t in g["t"]], "k:")
    axes[0, 1].set_title("Sintonia elegida aplicada a la planta real"); axes[0, 1].legend(fontsize=8)
    axes[1, 1].plot(g["t"] / 60, g["u"] * 100, color=COL["gemelo"])
    axes[1, 1].plot(rr["t"] / 60, rr["u"] * 100, color=COL["fisico"], lw=0.8)
    axes[1, 1].set_xlabel("Tiempo (min)")
    fig.suptitle("1. Puesta en marcha virtual: la agresiva apenas mejora el error pero triplica el desgaste "
                 "de la bomba -> se elige la moderada y se confirma en planta")
    fig.tight_layout(); guardar(fig, SALIDA, "1_puesta_en_marcha.png")
    print(f"  {'Sintonia':30s}{'error medio':>12s}{'sobrepaso':>11s}{'desgaste bomba':>16s}")
    for n, (e, s, d) in resumen.items():
        print(f"  {n:30s}{e:10.1f} cm{s:9.1f} cm{d:14.1f}")


def causal(x, n):
    """Media movil causal (como la haria el PLC en linea)."""
    c = np.cumsum(np.insert(x, 0, 0.0))
    out = np.empty_like(x)
    for k in range(len(x)):
        a = max(0, k - n + 1)
        out[k] = (c[k + 1] - c[a]) / (k + 1 - a)
    return out


def parte_b():
    T_FUGA = 5400
    r = correr(P_REAL, 4.0, 1800, T=9000, semilla=7, fuga_desde=T_FUGA)
    h1f = causal(r["h1_m"], 60)                                       # filtros CAUSALES:
    dh1 = np.concatenate([[0], np.diff(h1f)]) / DT                    # solo usan el pasado
    # 1) CALIBRAR el gemelo con datos normales (sin fuga):  dh1/dt = a*u - b*sqrt(h1)
    calib = slice(600, 5000)
    X = np.column_stack([r["u"][calib], -np.sqrt(np.maximum(h1f[calib], 0))])
    (a, b), *_ = np.linalg.lstsq(X, dh1[calib], rcond=None)
    print(f"  Calibracion del gemelo: k_b/A1 = {a:.5f} (nominal {P_NOM.k_b / P_NOM.A1:.5f}, "
          f"real {P_REAL.k_b / P_REAL.A1:.5f})")
    # 2) Balance de masa con el gemelo calibrado: lo que entra - lo que sale - lo que se acumula
    def fuga_estimada(aa, bb):
        q = P_NOM.A1 * (aa * r["u"] - bb * np.sqrt(np.maximum(h1f, 0)) - dh1)
        return causal(q, 900)                                          # promedio de 15 min
    q_nom = fuga_estimada(P_NOM.k_b / P_NOM.A1, P_NOM.c12 / P_NOM.A1)
    q_corr = fuga_estimada(a, b)
    umbral = 4 * q_corr[calib].std() + abs(q_corr[calib].mean())
    alarma = np.flatnonzero((q_corr > umbral) & (r["t"] > 600))
    t_alarma = r["t"][alarma[0]] if len(alarma) else np.nan
    falsas_nom = np.mean(np.abs(q_nom[calib]) > umbral) * 100
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6.5), sharex=True)
    ax1.plot(r["t"] / 60, r["h1"], color=COL["fisico"], label="nivel tanque 1")
    ax1.plot(r["t"] / 60, r["h2"], color=COL["gemelo"], label="nivel tanque 2")
    ax1.axvline(T_FUGA / 60, color=COL["falla"], ls=":", label="aparece la fuga")
    ax1.set_ylabel("m"); ax1.legend(fontsize=8)
    ax1.set_title("El controlador compensa la fuga: mirando los niveles, NADIE la nota")
    ax2.plot(r["t"] / 60, r["fuga"] * 1000, color=COL["fisico"], lw=2, label="fuga real")
    ax2.plot(r["t"] / 60, q_nom * 1000, color=COL["medido"], lw=1,
             label="gemelo SIN calibrar (parametros de catalogo)")
    ax2.plot(r["t"] / 60, q_corr * 1000, color=COL["gemelo"], label="gemelo calibrado")
    ax2.axhline(umbral * 1000, color=COL["falla"], ls="--", label="umbral")
    ax2.axvspan(calib.start / 60, calib.stop / 60, color=COL["ok"], alpha=0.07, label="datos de calibracion")
    if np.isfinite(t_alarma):
        ax2.axvline(t_alarma / 60, color=COL["falla"])
        ax2.text(t_alarma / 60 + 1, umbral * 1000 * 1.5, f"ALARMA a los {(t_alarma - T_FUGA) / 60:.0f} min",
                 color=COL["falla"])
    ax2.set_ylabel("Caudal (L/s)"); ax2.set_xlabel("Tiempo (min)"); ax2.legend(fontsize=7, ncol=2)
    fig.suptitle("2. Deteccion de fugas por balance de masa: solo funciona con el gemelo CALIBRADO")
    fig.tight_layout(); guardar(fig, SALIDA, "2_fuga.png")
    print(f"  Gemelo sin calibrar: fuera de umbral el {falsas_nom:.0f} % del tiempo SIN fuga (falsas alarmas)")
    print(f"  Fuga real {r['fuga'][-1] * 1000:.1f} L/s desde t={T_FUGA / 60:.0f} min | "
          f"detectada {(t_alarma - T_FUGA) / 60:.0f} min despues | estimada {q_corr[-600:-150].mean() * 1000:.1f} L/s")


def autonomia(h1, h2, t0, p=P_NOM, max_t=7200):
    """Simula en el gemelo: si la bomba falla AHORA, cuantos segundos hasta h2 < H_MIN."""
    sim = Tanques(p, h1, h2); sim.bomba_ok = False
    for k in range(int(max_t / 5)):
        sim.paso(0, consumo(t0 + k * 5), 5)
        if sim.h2 < H_MIN:
            return k * 5
    return max_t


def parte_c():
    r = correr(P_REAL, 4.0, 1800, T=7200, semilla=3)
    tiempos = r["t"][::120]
    aut = [autonomia(r["h1_m"][int(t)], r["h2_m"][int(t)], t) / 60 for t in tiempos]
    # validacion: la bomba falla de verdad a los 85 min (en plena demanda alta)
    T_F = 5100
    real = Tanques(P_REAL, r["h1"][T_F], r["h2"][T_F]); real.bomba_ok = False
    t_real = None
    for k in range(7200):
        real.paso(0, consumo(T_F + k), 1)
        if real.h2 < H_MIN:
            t_real = k / 60; break
    pred = autonomia(r["h1_m"][T_F], r["h2_m"][T_F], T_F) / 60
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(tiempos / 60, aut, "o-", color=COL["gemelo"], ms=3, label="autonomia calculada por el gemelo")
    ax.axhline(15, color=COL["alerta"], ls="--", label="minimo exigido: 15 min para reaccionar")
    ax2 = ax.twinx(); ax2.plot(r["t"] / 60, r["a"] * 100, color="gray", alpha=0.5); ax2.grid(False)
    ax2.set_ylabel("Consumo (% valvula)", color="gray")
    ax.plot(T_F / 60, pred, "s", color=COL["pred"], ms=10, label=f"prediccion en t={T_F / 60:.0f} min: {pred:.1f} min")
    if t_real:
        ax.plot(T_F / 60, t_real, "x", color=COL["falla"], ms=12, mew=3,
                label=f"falla real de la bomba: se vacio en {t_real:.1f} min")
    ax.set_xlabel("Tiempo (min)"); ax.set_ylabel("Minutos hasta vaciar el tanque 2")
    ax.legend(fontsize=8, loc="upper left")
    ax.set_title("3. Indicador '¿que pasa si...?' en vivo: autonomia ante falla de bomba")
    fig.tight_layout(); guardar(fig, SALIDA, "3_autonomia.png")
    print(f"\n  Falla de bomba a los {T_F / 60:.0f} min: el gemelo predijo {pred:.1f} min de autonomia, "
          f"la planta real duro {t_real:.1f} min")


def main():
    titulo("21.13 Digital Twin de un sistema industrial")
    print("  A) Puesta en marcha virtual")
    parte_a()
    print("\n  B) Deteccion de fugas")
    parte_b()
    print("\n  C) Analisis de autonomia")
    parte_c()
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
