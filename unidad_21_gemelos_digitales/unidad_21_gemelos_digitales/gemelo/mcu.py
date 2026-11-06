"""
Microcontrolador virtual: lo justo para que el gemelo "piense" como el firmware real.

  ADC   cuantizacion a N bits con referencia Vref
  PWM   registro de comparacion de N bits
  Planificador ciclico con tiempos de ejecucion, jitter y deteccion de overrun
  Telemetria UART: tramas binarias con cabecera y checksum
"""
import struct

import numpy as np


class ADC:
    def __init__(self, bits=12, vref=3.3, ruido_lsb=0.5, semilla=None):
        self.bits, self.vref, self.ruido = bits, vref, ruido_lsb
        self.max = 2 ** bits - 1
        self.rng = np.random.default_rng(semilla)

    def convertir(self, voltios):
        codigo = voltios / self.vref * self.max + self.rng.normal(0, self.ruido)
        return int(np.clip(round(codigo), 0, self.max))

    def a_voltios(self, codigo):
        return codigo * self.vref / self.max


class PWM:
    def __init__(self, bits=10, frecuencia=20e3):
        self.bits, self.f = bits, frecuencia
        self.top = 2 ** bits - 1

    def registro(self, duty):
        return int(round(np.clip(duty, 0, 1) * self.top))

    def duty(self, registro):
        return registro / self.top


class Tarea:
    def __init__(self, nombre, periodo, ciclos, variacion=0.1, prioridad=0):
        self.nombre, self.periodo, self.ciclos = nombre, periodo, ciclos
        self.variacion, self.prioridad = variacion, prioridad


class MCUVirtual:
    def __init__(self, f_cpu=72e6, tick=1e-3, latencia_isr_ciclos=(20, 400), semilla=0):
        self.f_cpu, self.tick = f_cpu, tick
        self.lat_isr = latencia_isr_ciclos
        self.tareas = []
        self.rng = np.random.default_rng(semilla)

    def agregar_tarea(self, tarea):
        self.tareas.append(tarea)
        self.tareas.sort(key=lambda t: t.prioridad)

    def simular(self, duracion):
        """Ejecutivo ciclico: en cada tick corre las tareas que tocan, en orden de prioridad.
        Devuelve, por tick: carga de CPU, retardo de la tarea de control, overrun."""
        n = int(duracion / self.tick)
        carga, retardo_ctrl, overruns, eventos = np.zeros(n), np.zeros(n), np.zeros(n, bool), []
        pendiente = 0.0                      # trabajo que se arrastra del tick anterior
        for k in range(n):
            t0 = k * self.tick
            inicio = self.rng.uniform(*self.lat_isr) / self.f_cpu + pendiente
            t_rel = inicio
            for tarea in self.tareas:
                if (k * self.tick + 1e-12) % tarea.periodo < self.tick:
                    dur = tarea.ciclos * (1 + self.rng.uniform(0, tarea.variacion)) / self.f_cpu
                    if tarea.prioridad == 0:
                        retardo_ctrl[k] = t_rel
                    if k < 40:
                        eventos.append((tarea.nombre, t0 + t_rel, dur))
                    t_rel += dur
            carga[k] = t_rel / self.tick
            overruns[k] = t_rel > self.tick
            pendiente = max(0.0, t_rel - self.tick)
        return dict(carga=carga, retardo=retardo_ctrl, overruns=overruns, eventos=eventos)


# ---------------------------------------------------------------------
# Telemetria: trama binaria   AA 55 | seq u16 | t_ms u32 | w i16 | i i16 | duty u16 | T i16 | xor
# ---------------------------------------------------------------------
FORMATO = "<HHIhhHh"


def empaquetar_trama(seq, t_ms, w, i, duty, T):
    cuerpo = struct.pack(FORMATO, 0x55AA, seq & 0xFFFF, int(t_ms) & 0xFFFFFFFF,
                         int(np.clip(w * 10, -32768, 32767)),     # 0.1 rad/s
                         int(np.clip(i * 1000, -32768, 32767)),   # mA
                         int(np.clip(duty * 10000, 0, 65535)),    # 0.01 %
                         int(np.clip(T * 10, -32768, 32767)))     # 0.1 C
    chk = 0
    for b in cuerpo:
        chk ^= b
    return cuerpo + bytes([chk])


def desempaquetar_trama(trama):
    if len(trama) != struct.calcsize(FORMATO) + 1:
        return None
    chk = 0
    for b in trama[:-1]:
        chk ^= b
    if chk != trama[-1]:
        return None
    cab, seq, t_ms, w, i, d, T = struct.unpack(FORMATO, trama[:-1])
    if cab != 0x55AA:
        return None
    return dict(seq=seq, t=t_ms / 1000, w=w / 10, i=i / 1000, duty=d / 10000, T=T / 10)


TAMANO_TRAMA = struct.calcsize(FORMATO) + 1
