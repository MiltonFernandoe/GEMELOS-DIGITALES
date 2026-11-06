"""
21.2 Arquitectura de un Digital Twin
====================================
Implementa una tuberia de 6 capas y mide cuantos datos y cuanta latencia
agrega cada una:

  1. Fisica        motor + sensores a 1 kHz
  2. Edge (MCU)    decimacion a 50 Hz + envio por banda muerta (solo si cambia)
  3. Comunicacion  red con retardo y perdidas (tipo MQTT sobre WiFi)
  4. Datos         almacenamiento de series de tiempo (buffer circular)
  5. Modelo        gemelo: reconstruye la senal y calcula indicadores
  6. Servicios     alarmas y tablero

Genera: 1_capas.png, 2_volumen_datos.png, 3_latencia.png, 4_banda_muerta.png
"""
import sys
from collections import deque
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.mcu import TAMANO_TRAMA, desempaquetar_trama, empaquetar_trama
from gemelo.motor import MotorDC
from gemelo.red import CanalRed
from gemelo.sensores import Sensor

SALIDA = carpeta_resultados(__file__)
DT, DURACION = 1e-3, 30.0


def diagrama():
    capas = [("6. Servicios", "tableros, alarmas, ordenes de trabajo, API", "#fde0dd"),
             ("5. Modelo / Gemelo", "fisica + datos + ML, sincronizacion, prediccion", "#dbe9f6"),
             ("4. Datos", "series de tiempo, historial, metadatos del activo", "#e5f5e0"),
             ("3. Comunicacion", "MQTT / OPC UA / CoAP / LoRaWAN, seguridad TLS", "#fff2cc"),
             ("2. Edge (MCU)", "adquisicion, filtrado, compresion, control local", "#ede7f6"),
             ("1. Fisica", "motor, sensores, actuadores", "#eeeeee")]
    fig, ax = plt.subplots(figsize=(11, 7))
    ax.set_xlim(0, 12); ax.set_ylim(0, 13); ax.axis("off")
    for k, (nombre, desc, col) in enumerate(capas):
        y = 11 - 2 * k
        ax.add_patch(FancyBboxPatch((1, y), 8, 1.5, boxstyle="round,pad=0.1", fc=col, ec="k"))
        ax.text(1.3, y + 0.95, nombre, fontsize=11, fontweight="bold", va="center")
        ax.text(1.3, y + 0.45, desc, fontsize=9, va="center")
        if k < len(capas) - 1:
            ax.add_patch(FancyArrowPatch((4, y - 0.45), (4, y - 0.05), arrowstyle="<|-", mutation_scale=14,
                                         color=COL["gemelo"], lw=2))
            ax.add_patch(FancyArrowPatch((6, y - 0.45), (6, y - 0.05), arrowstyle="-|>", mutation_scale=14,
                                         color=COL["ok"], lw=2))
    ax.text(3.0, 12.7, "datos suben", color=COL["gemelo"], fontsize=10)
    ax.text(5.6, 12.7, "decisiones bajan", color=COL["ok"], fontsize=10)
    ax.add_patch(FancyBboxPatch((9.5, 1), 1.8, 11.5, boxstyle="round,pad=0.1", fc="#f7f7f7", ec="gray"))
    ax.text(10.4, 6.8, "Transversal:\n\nseguridad\n\nidentidad\ndel activo\n\nversionado\nde modelos\n\nsincronizacion\nde tiempo",
            ha="center", va="center", fontsize=9)
    ax.set_title("1. Arquitectura en capas de un gemelo digital (inspirada en ISO 23247 / RAMI 4.0)")
    guardar(fig, SALIDA, "1_capas.png")


def main():
    titulo("21.2 Arquitectura de un Digital Twin")
    diagrama()
    rng = np.random.default_rng(0)

    # --- 1. Capa fisica
    motor = MotorDC()
    s_w, s_i = Sensor(ruido=0.3, semilla=1), Sensor(ruido=0.01, semilla=2)
    # --- 2. Edge
    DECIM, BANDA_MUERTA, LATIDO = 20, 3.0, 1.0       # 50 Hz, 3 rad/s, 1 trama/s minimo
    ultimo_enviado, t_ultimo, seq = None, -1.0, 0
    acum_w, acum_i = 0.0, 0.0
    # --- 3. Comunicacion
    red = CanalRed(retardo=0.040, jitter=0.020, perdida=0.01, semilla=3)
    # --- 4. Datos
    almacen = deque(maxlen=10_000)
    # --- 5/6. Modelo y servicios
    alarmas = []
    bytes_capa = {"1. Fisica (crudo 1 kHz)": 0, "2. Edge decimado 50 Hz": 0, "2. Edge + banda muerta": 0}
    lat = {"muestreo/edge": [], "red": [], "ingesta": [], "modelo": [], "total": []}
    log_real, log_rec = [], []

    n = int(DURACION / DT)
    for k in range(n):
        t = k * DT
        V = 12.0 if (t % 8) < 4 else 6.0 + 4 * np.sin(2 * np.pi * 0.3 * t)
        carga = 0.02 if t < 15 else 0.05
        motor.paso(V, DT, carga)
        w, i = s_w.medir(motor.w), s_i.medir(motor.i)
        bytes_capa["1. Fisica (crudo 1 kHz)"] += 2 * 2        # dos senales int16
        acum_w += w; acum_i += i
        if k % 10 == 0:
            log_real.append((t, motor.w))
        if (k + 1) % DECIM == 0:                               # edge: promedio de 20 muestras
            w_m, i_m = acum_w / DECIM, acum_i / DECIM
            acum_w = acum_i = 0.0
            bytes_capa["2. Edge decimado 50 Hz"] += TAMANO_TRAMA
            cambio = ultimo_enviado is None or abs(w_m - ultimo_enviado) > BANDA_MUERTA
            if cambio or t - t_ultimo >= LATIDO:
                trama = empaquetar_trama(seq, t * 1000, w_m, i_m, V / 12, motor.T)
                t_listo = t + 0.0005                           # 0.5 ms de procesamiento en el MCU
                red.enviar((trama, t - DECIM * DT / 2, t_listo), t_listo)
                bytes_capa["2. Edge + banda muerta"] += TAMANO_TRAMA
                ultimo_enviado, t_ultimo, seq = w_m, t, seq + 1
        for t_llega, (trama, t_medio, t_listo) in red.recibir(t):
            d = desempaquetar_trama(trama)
            t_ingesta = t_llega + rng.uniform(0.001, 0.004)      # cola del broker + base de datos
            almacen.append(d)
            t_modelo = t_ingesta + 0.002                         # el gemelo procesa
            log_rec.append((t_modelo, d["w"]))
            if d["i"] > 1.3:
                alarmas.append(t_modelo)
            lat["muestreo/edge"].append(t_listo - t_medio)
            lat["red"].append(t_llega - t_listo)
            lat["ingesta"].append(t_ingesta - t_llega)
            lat["modelo"].append(t_modelo - t_ingesta)
            lat["total"].append(t_modelo - t_medio)

    print(f"  Tramas enviadas: {red.enviados}, perdidas en la red: {red.perdidos}")
    for c, b in bytes_capa.items():
        print(f"  {c:28s}: {b / DURACION:8.0f} bytes/s")
    print(f"  Latencia total media: {np.mean(lat['total']) * 1000:.1f} ms  "
          f"(p95 {np.percentile(lat['total'], 95) * 1000:.1f} ms)")

    # --- 2. volumen de datos
    fig, ax = plt.subplots(figsize=(9, 4))
    nombres, vals = list(bytes_capa), [b / DURACION for b in bytes_capa.values()]
    ax.bar(nombres, vals, color=["#999999", COL["gemelo"], COL["ok"]])
    for k, v in enumerate(vals):
        ax.text(k, v, f"{v:,.0f} B/s", ha="center", va="bottom")
    ax.set_yscale("log"); ax.set_ylabel("Bytes por segundo (log)")
    ax.set_title(f"2. El edge reduce {vals[0] / vals[-1]:.0f}x los datos antes de salir del dispositivo")
    fig.tight_layout(); guardar(fig, SALIDA, "2_volumen_datos.png")

    # --- 3. latencia por capa
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4))
    etapas = ["muestreo/edge", "red", "ingesta", "modelo"]
    medias = [np.mean(lat[e]) * 1000 for e in etapas]
    izq = 0
    for e, m, c in zip(etapas, medias, ["#ede7f6", "#fff2cc", "#e5f5e0", "#dbe9f6"]):
        ax1.barh(0, m, left=izq, color=c, edgecolor="k")
        ax1.text(izq + m / 2, 0, f"{e}\n{m:.1f} ms", ha="center", va="center", fontsize=8)
        izq += m
    ax1.set_yticks([]); ax1.set_xlabel("ms"); ax1.set_title("Latencia media por capa")
    ax2.hist(np.array(lat["total"]) * 1000, bins=40, color=COL["gemelo"])
    ax2.axvline(np.percentile(lat["total"], 95) * 1000, color=COL["falla"], ls="--", label="percentil 95")
    ax2.set_xlabel("Latencia extremo a extremo (ms)"); ax2.legend(); ax2.set_title("Distribucion")
    fig.suptitle("3. Cuanto tarda un dato en llegar desde el sensor hasta el gemelo")
    fig.tight_layout(); guardar(fig, SALIDA, "3_latencia.png")

    # --- 4. reconstruccion con banda muerta
    real, rec = np.array(log_real), np.array(log_rec)
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(real[:, 0], real[:, 1], color=COL["fisico"], lw=1, label="velocidad real (1 kHz)")
    ax.step(rec[:, 0], rec[:, 1], where="post", color=COL["gemelo"], lw=1.3,
            label=f"lo que ve el gemelo ({len(rec)} tramas)")
    ax.plot(rec[:, 0], rec[:, 1], ".", color=COL["gemelo"], ms=3)
    ax.set_xlim(2, 12); ax.set_xlabel("Tiempo (s)"); ax.set_ylabel("rad/s"); ax.legend()
    ax.set_title("4. Envio por banda muerta: muchas tramas cuando cambia, pocas cuando esta estable")
    fig.tight_layout(); guardar(fig, SALIDA, "4_banda_muerta.png")
    print(f"  Alarmas de sobrecorriente generadas por la capa de servicios: {len(alarmas)}")
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
