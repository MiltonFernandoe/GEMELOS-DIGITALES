"""
21.4 Representacion virtual de un microcontrolador
==================================================
El gemelo no solo modela el motor: tambien modela al MCU que lo controla,
porque sus limitaciones cambian el comportamiento del sistema:

  A) ADC de N bits      -> resolucion de la medicion y ruido en el control
  B) PWM a f Hz         -> rizado de corriente en el motor
  C) Planificador       -> carga de CPU, jitter y overruns del lazo de control
  D) Telemetria UART    -> ancho de banda y deteccion de errores con checksum

Genera: 1_adc.png, 2_pwm.png, 3_planificador.png, 4_telemetria.png
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.mcu import (ADC, PWM, TAMANO_TRAMA, MCUVirtual, Tarea, desempaquetar_trama,
                        empaquetar_trama)
from gemelo.motor import MotorDC, ParamsMotor
from gemelo.sensores import PuenteH

SALIDA = carpeta_resultados(__file__)


def firmware_pi(adc, pwm, w_ref, tacometro_v, estado, dt, kp=0.004, ki=0.08):
    """Lo que corre en el MCU: leer ADC -> PI -> escribir registro PWM."""
    codigo = adc.convertir(tacometro_v)
    w_med = adc.a_voltios(codigo) / 3.3 * 300.0          # tacometro: 0..3.3 V = 0..300 rad/s
    e = w_ref - w_med
    estado["I"] = np.clip(estado["I"] + ki * e * dt, 0, 1)
    duty = np.clip(kp * e + estado["I"], 0, 1)
    return pwm.registro(duty), w_med


def lazo_cerrado(bits_adc, T=2.0, dt=1e-3):
    motor, adc, pwm, puente = MotorDC(), ADC(bits_adc, semilla=1), PWM(10), PuenteH(zona_muerta=0.0)
    estado = {"I": 0.0}
    t, W, D = np.arange(int(T / dt)) * dt, [], []
    for tk in t:
        w_ref = 150.0 if tk < 1.0 else 152.0             # escalon pequeno de 2 rad/s
        reg, _ = firmware_pi(adc, pwm, w_ref, motor.w / 300 * 3.3, estado, dt)
        motor.paso(puente.aplicar(pwm.duty(reg)), dt, 0.01)
        W.append(motor.w); D.append(pwm.duty(reg))
    return t, np.array(W), np.array(D)


def main():
    titulo("21.4 Representacion virtual de un microcontrolador")

    # --- A. ADC
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.5))
    v = np.linspace(1.0, 1.1, 400)                      # zoom en 100 mV
    for bits, c in [(8, COL["modelo"]), (10, COL["alerta"]), (12, COL["gemelo"])]:
        adc = ADC(bits, ruido_lsb=0.0)
        axes[0].step(v * 1000, [adc.a_voltios(adc.convertir(x)) * 1000 for x in v], where="mid",
                     color=c, label=f"{bits} bits: 1 LSB = {3.3 / 2 ** bits * 1000:.2f} mV")
    axes[0].plot(v * 1000, v * 1000, "k--", lw=0.8)
    axes[0].set_xlabel("Tension real (mV)"); axes[0].set_ylabel("Lectura (mV)"); axes[0].legend(fontsize=8)
    axes[0].set_title("Cuantizacion del ADC (zoom de 100 mV)")
    for bits, c in [(8, COL["modelo"]), (10, COL["alerta"]), (12, COL["gemelo"])]:
        t, W, D = lazo_cerrado(bits)
        axes[1].plot(t, W, color=c, lw=1, label=f"{bits} bits: rizado {np.std(W[1200:]):.2f} rad/s")
        print(f"  ADC {bits:2d} bits -> resolucion {300 / 2 ** bits:5.2f} rad/s | "
              f"desv. velocidad {np.std(W[1200:]):.3f} rad/s | desv. duty {np.std(D[1200:]) * 100:.2f} %")
    axes[1].set_xlim(0.8, 2.0); axes[1].set_ylim(146, 156)
    axes[1].set_xlabel("Tiempo (s)"); axes[1].set_ylabel("Velocidad (rad/s)"); axes[1].legend(fontsize=8)
    axes[1].set_title("Escalon de 2 rad/s: lo siguen los tres (el ruido actua como dithering)\n"
                      "pero con 8 bits el mando PWM oscila 7x mas")
    fig.suptitle("1. El ADC del gemelo debe tener la misma resolucion que el real")
    fig.tight_layout(); guardar(fig, SALIDA, "1_adc.png")

    # --- B. PWM y rizado de corriente
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    dt_fino = 1e-6
    for ax, f_pwm in zip(axes, [2e3, 20e3]):
        motor = MotorDC(); motor.w = 120.0               # motor ya girando
        motor.i = (0.6 * 12.0 - motor.p.Kt * 120.0) / motor.p.R0   # corriente media
        n_per = int(round(1 / (f_pwm * dt_fino)))       # contador del timer (como el MCU)
        n_on = int(round(0.6 * n_per))
        for k in range(int(8e-3 / dt_fino)):            # 8 ms para llegar a regimen
            tk = k * dt_fino
            motor.paso(12.0 if k % n_per < n_on else 0.0, dt_fino, 0.02); motor.w = 120.0
        t, I, Vs = [], [], []
        for k in range(int(3e-3 / dt_fino)):
            tk = k * dt_fino
            V = 12.0 if k % n_per < n_on else 0.0       # duty 60 %
            motor.paso(V, dt_fino, 0.02); motor.w = 120.0
            t.append(tk); I.append(motor.i); Vs.append(V)
        I = np.array(I); t = np.array(t)
        ax.plot(t * 1000, I, color=COL["gemelo"], label="corriente")
        ax2 = ax.twinx(); ax2.fill_between(t * 1000, Vs, step="mid", color="gray", alpha=0.15)
        ax2.set_ylabel("PWM (V)"); ax2.grid(False)
        rizado = I.max() - I.min()
        ax.set_title(f"PWM a {f_pwm / 1000:.0f} kHz: rizado de corriente = {rizado * 1000:.0f} mA pico a pico")
        ax.set_ylabel("i (A)")
        teo = 12.0 * 0.6 * 0.4 / (motor.p.L * f_pwm)
        print(f"  PWM {f_pwm / 1000:4.0f} kHz -> rizado {rizado * 1000:5.0f} mA "
              f"(teorico V*d*(1-d)/(L*f) = {teo * 1000:.0f} mA)")
    axes[-1].set_xlabel("Tiempo (ms)")
    fig.suptitle("2. La frecuencia del PWM (un registro del MCU) cambia la corriente que ve el motor")
    fig.tight_layout(); guardar(fig, SALIDA, "2_pwm.png")

    # --- C. planificador: lazo de control + comunicacion + diagnostico FFT
    configs = {"72 MHz, FFT en un bloque": (72e6, [Tarea("FFT diagnostico", 0.1, 120_000, 0.05, 2)]),
               "72 MHz, FFT dividida en 10 partes": (72e6, [Tarea("FFT diagnostico", 0.01, 12_000, 0.05, 2)]),
               "168 MHz, FFT en un bloque": (168e6, [Tarea("FFT diagnostico", 0.1, 120_000, 0.05, 2)])}
    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(2, 3)
    for col, (nombre, (f_cpu, extra)) in enumerate(configs.items()):
        mcu = MCUVirtual(f_cpu=f_cpu, tick=1e-3, semilla=col)
        mcu.agregar_tarea(Tarea("control PI", 1e-3, 15_000, 0.1, 0))
        mcu.agregar_tarea(Tarea("telemetria", 0.01, 20_000, 0.2, 1))
        for tarea in extra:
            mcu.agregar_tarea(tarea)
        r = mcu.simular(2.0)
        ax = fig.add_subplot(gs[0, col])
        colores = {"control PI": COL["gemelo"], "telemetria": COL["ok"], "FFT diagnostico": COL["alerta"]}
        filas = {"control PI": 2, "telemetria": 1, "FFT diagnostico": 0}
        for n, t0, d in r["eventos"]:
            if t0 < 0.012:
                ax.barh(filas[n], d * 1000, left=t0 * 1000, color=colores[n], edgecolor="k", lw=0.3)
        for k in range(13):
            ax.axvline(k, color="gray", lw=0.5, alpha=0.5)
        ax.set_yticks([0, 1, 2]); ax.set_yticklabels(["FFT", "telemetria", "control"])
        ax.set_xlim(0, 12); ax.set_xlabel("ms (lineas = ticks de 1 ms)")
        ax.set_title(f"{nombre}\ncarga media {r['carga'].mean() * 100:.0f} %, overruns {r['overruns'].sum()}",
                     fontsize=9)
        ax = fig.add_subplot(gs[1, col])
        ax.hist(r["retardo"] * 1e6, bins=40, color=COL["gemelo"])
        ax.set_xlabel("Retardo del lazo de control (us)")
        ax.set_title(f"jitter: {np.ptp(r['retardo']) * 1e6:.0f} us", fontsize=9)
        print(f"  {nombre:35s}: carga {r['carga'].mean() * 100:4.1f} %, overruns {r['overruns'].sum():4d}, "
              f"jitter del control {np.ptp(r['retardo']) * 1e6:6.0f} us")
    fig.suptitle("3. Planificacion de tareas: una FFT larga retrasa al lazo de control (jitter y overruns)")
    fig.tight_layout(); guardar(fig, SALIDA, "3_planificador.png")

    # --- D. telemetria UART
    baud = 115200
    bytes_por_s = baud / 10                            # 8N1: 10 bits por byte
    tasas = [10, 50, 100, 500, 1000]
    uso = [TAMANO_TRAMA * f / bytes_por_s * 100 for f in tasas]
    rng = np.random.default_rng(0)
    ber_lista = [1e-5, 1e-4, 1e-3, 1e-2]
    detect, corrupt = [], []
    for ber in ber_lista:
        n_corr = n_det = 0
        for s in range(5000):
            trama = bytearray(empaquetar_trama(s, s, 100.0, 1.0, 0.5, 40.0))
            errores = rng.random(len(trama) * 8) < ber
            if errores.any():
                n_corr += 1
                for pos in np.flatnonzero(errores):
                    trama[pos // 8] ^= 1 << (pos % 8)
                n_det += desempaquetar_trama(bytes(trama)) is None
        corrupt.append(n_corr); detect.append(n_det / max(n_corr, 1) * 100)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.2))
    ax1.bar([str(f) for f in tasas], uso, color=[COL["ok"] if u < 70 else COL["falla"] for u in uso])
    ax1.axhline(70, color="k", ls="--", lw=0.8, label="limite practico 70 %")
    for k, u in enumerate(uso):
        ax1.text(k, u, f"{u:.0f} %", ha="center", va="bottom")
    ax1.set_xlabel("Tramas por segundo"); ax1.set_ylabel(f"Uso de UART {baud} baudios (%)")
    ax1.set_title(f"Trama de {TAMANO_TRAMA} bytes"); ax1.legend()
    ax2.bar([f"{b:g}" for b in ber_lista], detect, color=COL["gemelo"])
    for k, (d, c) in enumerate(zip(detect, corrupt)):
        ax2.text(k, d, f"{d:.1f} %\n({c} corruptas)", ha="center", va="bottom", fontsize=8)
    ax2.set_ylim(0, 115); ax2.set_xlabel("Tasa de error de bit (BER)")
    ax2.set_ylabel("Tramas corruptas detectadas (%)"); ax2.set_title("Checksum XOR de 1 byte")
    fig.suptitle("4. Telemetria: el gemelo debe saber cuanto puede pedirle al canal y descartar tramas danadas")
    fig.tight_layout(); guardar(fig, SALIDA, "4_telemetria.png")
    print(f"  Checksum XOR: detecta {min(detect):.1f}-{max(detect):.1f} % de las tramas corruptas")
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
