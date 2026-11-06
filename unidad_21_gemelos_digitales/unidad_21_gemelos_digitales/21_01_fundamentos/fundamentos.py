"""
21.1 Fundamentos: modelo digital vs sombra digital vs gemelo digital
====================================================================
Un motor mueve una bomba 10 minutos y la demanda aumenta sin que nadie lo avise.
Comparamos los tres niveles de integracion fisico <-> virtual:

  MODELO DIGITAL : simulacion aislada, no recibe datos del motor real.
  SOMBRA DIGITAL : recibe datos del motor (fisico -> virtual) y se actualiza.
  GEMELO DIGITAL : ademas ACTUA sobre el motor (virtual -> fisico):
                   predice que se va a sobrecalentar y reduce la tension.

Genera: 1_niveles.png (diagrama) y 2_tres_niveles.png (simulacion)
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.motor import MotorDC, ParamsMotor
from gemelo.sensores import Sensor

SALIDA = carpeta_resultados(__file__)
DT, DURACION = 0.5, 600.0
P_NOM = ParamsMotor()                               # lo que cree el modelo
P_REAL = P_NOM.copia(Rth=6.5, R0=2.05)              # el motor real difiere un poco
T_LIMITE, T_ALERTA = P_NOM.T_max, 110.0


W0 = 200.0   # rad/s de referencia para la carga tipo bomba


def carga_real(t):
    """Carga tipo bomba: par = k * w^2. k sube (mas caudal pedido) sin avisar al modelo."""
    par_a_w0 = 0.15 if t < 200 else (0.30 if t < 400 else 0.60)
    return par_a_w0 / W0 ** 2


def diagrama():
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    datos = [("Modelo digital", "manual / ninguno", "manual / ninguno", "#bbbbbb", "#bbbbbb"),
             ("Sombra digital", "automatico", "manual / ninguno", COL["gemelo"], "#bbbbbb"),
             ("Gemelo digital", "automatico", "automatico", COL["gemelo"], COL["ok"])]
    for ax, (nombre, subida, bajada, c_sub, c_baj) in zip(axes, datos):
        ax.set_xlim(0, 10); ax.set_ylim(0, 10); ax.axis("off")
        for y, txt, col in [(7.2, "Objeto VIRTUAL\n(modelo en Python)", "#dbe9f6"),
                            (0.8, "Objeto FISICO\n(motor + MCU)", "#f2f2f2")]:
            ax.add_patch(FancyBboxPatch((1.5, y), 7, 2, boxstyle="round,pad=0.2", fc=col, ec="k"))
            ax.text(5, y + 1, txt, ha="center", va="center", fontsize=10)
        ax.add_patch(FancyArrowPatch((3.5, 3.1), (3.5, 6.9), arrowstyle="-|>", mutation_scale=20,
                                     lw=2.5, color=c_sub))
        ax.text(3.3, 5, f"datos\n{subida}", ha="right", va="center", fontsize=8, color=c_sub)
        ax.add_patch(FancyArrowPatch((6.5, 6.9), (6.5, 3.1), arrowstyle="-|>", mutation_scale=20,
                                     lw=2.5, color=c_baj))
        ax.text(6.7, 5, f"acciones\n{bajada}", ha="left", va="center", fontsize=8, color=c_baj)
        ax.set_title(nombre, fontsize=13, fontweight="bold")
    fig.suptitle("1. Tres niveles de integracion (clasificacion de Kritzinger et al., 2018)")
    guardar(fig, SALIDA, "1_niveles.png")


def simular(nivel):
    fisico = MotorDC(P_REAL)
    virtual = MotorDC(P_NOM)
    s_T = Sensor("T", ruido=0.5, semilla=1)
    s_i = Sensor("i", ruido=0.02, semilla=2)
    s_w = Sensor("w", ruido=0.5, semilla=3)
    V, k_est = 12.0, 0.15 / W0 ** 2
    reg = {k: [] for k in ["t", "T_fis", "T_vir", "V", "pred"]}
    for k in range(int(DURACION / DT)):
        t = k * DT
        fisico.paso_lento(V, DT, k_cuad=carga_real(t))
        pred = np.nan
        if nivel == "modelo":
            virtual.paso_lento(V, DT, k_cuad=0.15 / W0 ** 2)   # supone la carga de diseno
        else:
            if k % 10 == 0:                               # llega telemetria cada 5 s
                T_m, i_m, w_m = s_T.medir(fisico.T), s_i.medir(fisico.i), s_w.medir(fisico.w)
                p = P_NOM
                par = max(0.0, p.Kt * i_m - p.b * w_m - p.Tc)   # sensor virtual de par de carga
                k_est = par / max(w_m, 1.0) ** 2                  # identifica la curva de la bomba
                virtual.T = T_m                           # sincroniza el estado
            virtual.paso_lento(V, DT, k_cuad=k_est)
            if nivel == "gemelo" and k % 10 == 0:
                # prediccion a 60 s con la tension actual
                prueba = MotorDC(P_NOM); prueba.T = virtual.T
                for _ in range(int(60 / DT)):
                    prueba.paso_lento(V, DT, k_cuad=k_est)
                pred = prueba.T
                if pred > T_ALERTA:
                    V = max(6.0, V - 1.0)                 # derating: baja la tension
                elif pred < T_ALERTA - 10 and V < 12.0:
                    V = min(12.0, V + 0.25)
        for key, val in zip(reg, [t, fisico.T, virtual.T, V, pred]):
            reg[key].append(val)
    return {k: np.array(v) for k, v in reg.items()}


def main():
    titulo("21.1 Fundamentos de gemelos digitales")
    diagrama()
    resultados = {n: simular(n) for n in ["modelo", "sombra", "gemelo"]}
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    nombres = {"modelo": "MODELO DIGITAL: no se entera de que la carga subio",
               "sombra": "SOMBRA DIGITAL: refleja la realidad, pero no evita el sobrecalentamiento",
               "gemelo": "GEMELO DIGITAL: predice el problema y reduce la tension a tiempo"}
    for ax, (n, r) in zip(axes, resultados.items()):
        ax.plot(r["t"], r["T_fis"], color=COL["fisico"], lw=2, label="motor fisico")
        ax.plot(r["t"], r["T_vir"], "--", color=COL["gemelo"], lw=1.5, label="representacion virtual")
        if n == "gemelo":
            ok = ~np.isnan(r["pred"])
            ax.plot(r["t"][ok], r["pred"][ok], ".", color=COL["pred"], ms=4, label="prediccion a 60 s")
            ax2 = ax.twinx(); ax2.plot(r["t"], r["V"], color=COL["ok"], lw=1.2, alpha=0.7)
            ax2.set_ylabel("Tension aplicada (V)", color=COL["ok"]); ax2.set_ylim(0, 14); ax2.grid(False)
        ax.axhline(T_LIMITE, color=COL["falla"], ls=":", label="limite aislamiento 130 C")
        for tc in (200, 400):
            ax.axvline(tc, color="gray", alpha=0.3)
        ax.set_ylabel("Temp. bobinado (C)"); ax.set_title(nombres[n]); ax.legend(loc="upper left", fontsize=8)
        print(f"  {n:7s}: temperatura maxima real = {r['T_fis'].max():6.1f} C | "
              f"error medio fisico-virtual = {np.mean(np.abs(r['T_fis'] - r['T_vir'])):5.1f} C")
    axes[-1].set_xlabel("Tiempo (s)   [lineas grises: la carga aumenta]")
    fig.suptitle("2. El mismo motor visto por los tres niveles de integracion")
    fig.tight_layout(); guardar(fig, SALIDA, "2_tres_niveles.png")
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
