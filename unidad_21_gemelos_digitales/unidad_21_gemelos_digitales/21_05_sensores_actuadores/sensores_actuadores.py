"""
21.5 Simulacion de sensores y actuadores
========================================
Un gemelo que asume sensores perfectos se equivoca. Aqui se simulan las
no-idealidades tipicas y su efecto:

  A) Galeria de defectos de sensor: ruido, offset/ganancia, cuantizacion,
     retardo, saturacion y deriva
  B) Encoder: medir velocidad contando pulsos vs midiendo el periodo
  C) Actuador (puente H): zona muerta, cuantizacion y limite de cambio,
     y como afectan a un lazo de control

Genera: 1_galeria_sensores.png, 2_encoder.png, 3_actuador.png
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.motor import MotorDC
from gemelo.sensores import PuenteH, Sensor

SALIDA = carpeta_resultados(__file__)


def galeria():
    t = np.linspace(0, 10, 1000)
    real = 40 + 30 * (1 - np.exp(-t / 2)) + 5 * np.sin(2 * np.pi * 0.3 * t)   # una temperatura
    casos = [
        ("Ruido (sigma = 1.5 C)", Sensor(ruido=1.5, semilla=1)),
        ("Offset +4 C y ganancia 0.92", Sensor(offset=4, ganancia=0.92)),
        ("Cuantizacion: ADC de 6 bits en 0-150 C", Sensor(rango=(0, 150), bits=6)),
        ("Retardo de 60 muestras (sensor lento)", Sensor(retardo=60)),
        ("Saturacion: rango maximo 70 C", Sensor(rango=(0, 70))),
        ("Deriva +0.8 C/s (envejecimiento)", Sensor(deriva=0.8)),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(15, 7), sharex=True, sharey=True)
    for ax, (nombre, s) in zip(axes.ravel(), casos):
        med = [s.medir(v, tk) for v, tk in zip(real, t)]
        ax.plot(t, real, color=COL["fisico"], lw=2, label="valor real")
        ax.plot(t, med, color=COL["modelo"], lw=1, label="medicion")
        ax.set_title(nombre)
    axes[0, 0].legend()
    for ax in axes[1]:
        ax.set_xlabel("Tiempo (s)")
    fig.suptitle("1. Seis formas en que un sensor 'miente'. El gemelo debe modelarlas para no confundirlas con el proceso")
    fig.tight_layout(); guardar(fig, SALIDA, "1_galeria_sensores.png")


def encoder():
    rng = np.random.default_rng(0)
    cpr = 400                                     # 100 ppr x4
    vel = np.logspace(0, 2.6, 40)                 # 1 a 400 rad/s
    metodos = {"conteo, Ts = 1 ms": [], "conteo, Ts = 10 ms": [], "periodo, timer 1 MHz": []}
    for w in vel:
        errs = {k: [] for k in metodos}
        for _ in range(200):
            fase = rng.random()
            for Ts, k in [(1e-3, "conteo, Ts = 1 ms"), (1e-2, "conteo, Ts = 10 ms")]:
                pulsos = np.floor(w * Ts * cpr / (2 * np.pi) + fase)
                errs[k].append(abs(pulsos * 2 * np.pi / cpr / Ts - w) / w)
            Tp = 2 * np.pi / cpr / w                  # tiempo entre flancos
            ticks = max(1, round(Tp * 1e6 + rng.uniform(-0.5, 0.5)))
            errs["periodo, timer 1 MHz"].append(abs(2 * np.pi / cpr / (ticks * 1e-6) - w) / w)
        for k in metodos:
            metodos[k].append(np.mean(errs[k]) * 100)
    fig, ax = plt.subplots(figsize=(9, 5))
    for (k, e), c in zip(metodos.items(), [COL["modelo"], COL["alerta"], COL["gemelo"]]):
        ax.loglog(vel, np.maximum(e, 1e-3), "o-", ms=3, color=c, label=k)
    ax.axhline(1, color="k", ls=":", lw=0.8)
    ax.set_xlabel("Velocidad real (rad/s)"); ax.set_ylabel("Error medio (%)")
    ax.set_title("2. Encoder de 400 cuentas/vuelta: contar pulsos falla a baja velocidad, medir el periodo falla a alta")
    ax.legend(); fig.tight_layout(); guardar(fig, SALIDA, "2_encoder.png")


def lazo(actuador, compensar=False, T=3.0, dt=1e-3):
    motor = MotorDC()
    I, t, W, Vs, R = 0.0, np.arange(int(T / dt)) * dt, [], [], []
    for tk in t:
        ref = 60.0 * np.sin(2 * np.pi * 0.7 * tk)       # avanza y retrocede
        e = ref - motor.w
        I = np.clip(I + 0.1 * e * dt, -1, 1)
        u = np.clip(0.004 * e + I, -1, 1)
        if compensar and abs(u) > 1e-3:                # compensacion de zona muerta en firmware
            u = np.sign(u) * (abs(u) * (1 - actuador.zm) + actuador.zm)
        V = actuador.aplicar(u, dt) if actuador else u * 12.0
        motor.paso(V, dt, 0.0)
        W.append(motor.w); Vs.append(V); R.append(ref)
    return t, np.array(W), np.array(Vs), np.array(R)


def actuador():
    casos = [("ideal", None, False, COL["fisico"]),
             ("puente H real: zona muerta 15 %, PWM 8 bits", PuenteH(zona_muerta=0.15, bits_pwm=8,
                                                                       max_cambio_por_s=20), False, COL["modelo"]),
             ("puente H real + compensacion en firmware", PuenteH(zona_muerta=0.15, bits_pwm=8,
                                                               max_cambio_por_s=20), True, COL["gemelo"])]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for nombre, act, comp, c in casos:
        t, W, V, ref = lazo(act, comp)
        rms = np.sqrt(np.mean((W[500:] - ref[500:]) ** 2))
        ax1.plot(t, W, color=c, lw=1.4, label=f"{nombre}  (error RMS {rms:.1f} rad/s)")
        ax2.plot(t, V, color=c, lw=1, alpha=0.8)
        print(f"  {nombre:52s}: error RMS de seguimiento = {rms:.2f} rad/s")
    ax1.plot(t, ref, "k:", lw=1, label="referencia")
    ax1.set_ylabel("Velocidad (rad/s)"); ax1.legend(fontsize=8)
    ax2.set_ylabel("Tension al motor (V)"); ax2.set_xlabel("Tiempo (s)")
    fig.suptitle("3. Zona muerta: el motor se queda quieto al cruzar por cero. La compensacion en firmware lo corrige")
    fig.tight_layout(); guardar(fig, SALIDA, "3_actuador.png")


def main():
    titulo("21.5 Simulacion de sensores y actuadores")
    galeria()
    encoder()
    actuador()
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
