"""
21.7 Integracion de datos en tiempo real
========================================
Simulacion EN TIEMPO REAL (reloj de pared) con tres hilos concurrentes:

  [hilo DISPOSITIVO] motor + MCU a 1 kHz, envia tramas binarias a 50 Hz
        |
  [hilo RED]         retardo variable, perdidas y desorden (como WiFi/MQTT)
        |
  [hilo GEMELO]      decodifica, reordena, detecta huecos y COMPENSA LA LATENCIA:
                     cada trama describe el pasado, el gemelo la proyecta al presente.

Dura ~8 s reales. Genera: 1_flujo.png, 2_latencia.png, 3_compensacion.png
"""
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.mcu import desempaquetar_trama, empaquetar_trama
from gemelo.motor import MotorDC, ParamsMotor
from gemelo.red import BufferReordenamiento, CanalRed
from gemelo.sensores import Sensor

SALIDA = carpeta_resultados(__file__)
DURACION = 8.0          # segundos reales
DT = 1e-3
TELEMETRIA = 0.02       # 50 Hz


def tension(t):
    return 12.0 if int(t / 0.6) % 2 == 0 else 5.0      # cambios bruscos: la latencia importa


class Compartido:
    def __init__(self):
        self.t0 = time.perf_counter()
        self.fin = threading.Event()
        self.lock = threading.Lock()
        self.verdad = []                                # (t, w) del motor real, para evaluar
        self.red = CanalRed(retardo=0.08, jitter=0.04, perdida=0.05, semilla=1)
        self.entrega = queue.Queue()

    def ahora(self):
        return time.perf_counter() - self.t0


def hilo_dispositivo(c):
    motor = MotorDC(ParamsMotor(R0=2.1, b=3e-5))
    s_w, s_i = Sensor(ruido=0.5, semilla=2), Sensor(ruido=0.01, semilla=3)
    t_sim, seq, prox_tx = 0.0, 0, 0.0
    while not c.fin.is_set():
        objetivo = c.ahora()
        while t_sim < objetivo:                         # alcanzar al reloj real
            V = tension(t_sim)
            motor.paso(V, DT, 0.01); motor.T = 25.0
            t_sim += DT
            if t_sim >= prox_tx:
                trama = empaquetar_trama(seq, t_sim * 1000, s_w.medir(motor.w), s_i.medir(motor.i),
                                         V / 12, motor.T)
                with c.lock:
                    c.red.enviar(trama, t_sim)
                seq += 1
                prox_tx += TELEMETRIA
            if int(t_sim * 1000) % 5 == 0:
                with c.lock:
                    c.verdad.append((t_sim, motor.w))
        time.sleep(0.002)


def hilo_red(c):
    while not c.fin.is_set():
        with c.lock:
            llegadas = c.red.recibir(c.ahora())
        for t_llegada, trama in llegadas:
            c.entrega.put((t_llegada, trama))
        time.sleep(0.001)


def hilo_gemelo(c, registro):
    buffer = BufferReordenamiento()
    modelo = MotorDC(ParamsMotor())                     # el gemelo usa parametros nominales
    while not c.fin.is_set():
        try:
            t_llegada, trama = c.entrega.get(timeout=0.05)
        except queue.Empty:
            continue
        d = desempaquetar_trama(trama)
        if d is None:
            continue
        registro["llegadas"].append((c.ahora(), d["seq"], d["t"]))
        for m in buffer.agregar(d["seq"], d):
            t_proc = c.ahora()
            edad = t_proc - m["t"]
            # Sin compensar: mostrar el ultimo valor recibido tal cual (dato viejo)
            # Compensado: arrancar el modelo desde el dato y simular hasta "ahora"
            modelo.i, modelo.w, modelo.T = m["i"], m["w"], 25.0
            pasos = int(edad / DT)
            for k in range(pasos):
                t_k = m["t"] + k * DT
                modelo.paso(tension(t_k), DT, 0.01)    # el gemelo conoce el plan de mando
                modelo.T = 25.0
            registro["proc"].append((t_proc, m["t"], m["w"], modelo.w, edad))
    registro["buffer"] = buffer


def main():
    titulo("21.7 Integracion de datos en tiempo real")
    print(f"  Corriendo {DURACION:.0f} s en tiempo real con 3 hilos...")
    c = Compartido()
    registro = {"llegadas": [], "proc": []}
    hilos = [threading.Thread(target=hilo_dispositivo, args=(c,)),
             threading.Thread(target=hilo_red, args=(c,)),
             threading.Thread(target=hilo_gemelo, args=(c, registro))]
    for h in hilos:
        h.start()
    time.sleep(DURACION)
    c.fin.set()
    for h in hilos:
        h.join()

    verdad = np.array(c.verdad)
    proc = np.array(registro["proc"])
    lleg = np.array(registro["llegadas"])
    buf = registro["buffer"]
    w_real_en = np.interp(proc[:, 0], verdad[:, 0], verdad[:, 1])
    err_sin = np.abs(proc[:, 2] - w_real_en)
    err_con = np.abs(proc[:, 3] - w_real_en)
    print(f"  Tramas enviadas {c.red.enviados}, perdidas en red {c.red.perdidos}, "
          f"llegaron desordenadas {buf.desordenados}, huecos detectados {buf.huecos}")
    print("  (huecos > perdidas: algunas tramas llegaron tan tarde que el buffer ya las habia descartado)")
    print(f"  Edad media del dato al procesarlo: {proc[:, 4].mean() * 1000:.0f} ms "
          f"(max {proc[:, 4].max() * 1000:.0f} ms)")
    print(f"  Error del gemelo SIN compensar latencia: {err_sin.mean():6.2f} rad/s (max {err_sin.max():.1f})")
    print(f"  Error del gemelo CON compensacion:       {err_con.mean():6.2f} rad/s (max {err_con.max():.1f})")

    # --- 1. flujo de mensajes
    fig, ax = plt.subplots(figsize=(11, 4.5))
    orden = np.diff(lleg[:, 1], prepend=-1) < 0
    ax.plot(lleg[:, 0], lleg[:, 1], ".", color=COL["gemelo"], ms=4, label="trama recibida")
    ax.plot(lleg[orden, 0], lleg[orden, 1], "o", mfc="none", color=COL["falla"], ms=8,
            label="llego despues de una mas nueva")
    ax.set_xlabel("Instante de llegada (s)"); ax.set_ylabel("Numero de secuencia")
    ax.set_title(f"1. Flujo de tramas: {c.red.perdidos} perdidas y {buf.desordenados} desordenadas; "
                 "el buffer las reordena por secuencia")
    ax.set_xlim(2, 4); ax.set_ylim(lleg[(lleg[:, 0] > 2) & (lleg[:, 0] < 4), 1].min() - 2,
                                   lleg[(lleg[:, 0] > 2) & (lleg[:, 0] < 4), 1].max() + 2)
    ax.legend(); fig.tight_layout(); guardar(fig, SALIDA, "1_flujo.png")

    # --- 2. latencia
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.hist(proc[:, 4] * 1000, bins=40, color=COL["gemelo"])
    ax.axvline(np.percentile(proc[:, 4], 95) * 1000, color=COL["falla"], ls="--", label="p95")
    ax.set_xlabel("Edad del dato cuando el gemelo lo usa (ms)"); ax.legend()
    ax.set_title("2. Todo dato que llega al gemelo ya es del pasado")
    fig.tight_layout(); guardar(fig, SALIDA, "2_latencia.png")

    # --- 3. compensacion
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6.5), sharex=True)
    ax1.plot(verdad[:, 0], verdad[:, 1], color=COL["fisico"], lw=2, label="motor real (ahora)")
    ax1.step(proc[:, 0], proc[:, 2], where="post", color=COL["modelo"], label="gemelo: ultimo dato recibido")
    ax1.step(proc[:, 0], proc[:, 3], where="post", color=COL["gemelo"], label="gemelo: dato proyectado al presente")
    ax1.set_ylabel("rad/s"); ax1.legend(fontsize=8); ax1.set_xlim(1, 5)
    ax2.plot(proc[:, 0], err_sin, color=COL["modelo"], label=f"sin compensar (media {err_sin.mean():.1f})")
    ax2.plot(proc[:, 0], err_con, color=COL["gemelo"], label=f"compensado (media {err_con.mean():.1f})")
    ax2.set_ylabel("Error (rad/s)"); ax2.set_xlabel("Tiempo real (s)"); ax2.legend(fontsize=8)
    fig.suptitle("3. Compensacion de latencia: el gemelo usa su modelo para 'adelantar' cada dato hasta el presente")
    fig.tight_layout(); guardar(fig, SALIDA, "3_compensacion.png")
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
