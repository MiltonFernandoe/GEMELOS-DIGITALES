"""Canal de comunicacion dispositivo -> gemelo (tipo MQTT/WiFi/LoRa) con imperfecciones."""
import heapq

import numpy as np


class CanalRed:
    def __init__(self, retardo=0.05, jitter=0.03, perdida=0.02, semilla=0):
        self.retardo, self.jitter, self.perdida = retardo, jitter, perdida
        self.rng = np.random.default_rng(semilla)
        self.cola = []                      # (t_llegada, contador, mensaje)
        self.n = 0
        self.enviados = self.perdidos = 0

    def enviar(self, mensaje, t_envio):
        self.enviados += 1
        if self.rng.random() < self.perdida:
            self.perdidos += 1
            return
        t_llegada = t_envio + self.retardo + self.rng.exponential(self.jitter)
        heapq.heappush(self.cola, (t_llegada, self.n, mensaje))
        self.n += 1

    def recibir(self, t_ahora):
        """Devuelve los mensajes que ya llegaron (pueden venir desordenados)."""
        listos = []
        while self.cola and self.cola[0][0] <= t_ahora:
            t, _, m = heapq.heappop(self.cola)
            listos.append((t, m))
        return listos


class BufferReordenamiento:
    """Ordena por numero de secuencia y detecta huecos (mensajes perdidos)."""

    def __init__(self):
        self.esperado = 0
        self.pendientes = {}
        self.huecos = 0
        self.desordenados = 0

    def agregar(self, seq, mensaje):
        if seq < self.esperado:
            return []                        # duplicado o demasiado tarde
        if seq != self.esperado:
            self.desordenados += 1
        self.pendientes[seq] = mensaje
        salida = []
        while self.esperado in self.pendientes:
            salida.append(self.pendientes.pop(self.esperado))
            self.esperado += 1
        # si hay demasiados pendientes, se da por perdido el hueco
        if len(self.pendientes) > 5:
            siguiente = min(self.pendientes)
            self.huecos += siguiente - self.esperado
            self.esperado = siguiente
            salida += self.agregar(siguiente, self.pendientes.pop(siguiente))
        return salida
