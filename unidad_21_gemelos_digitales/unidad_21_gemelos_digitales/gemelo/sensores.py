"""
Sensores y actuadores con no-idealidades y fallas inyectables.

Fallas de sensor (atributo .falla = dict):
  {"tipo": "sesgo",   "valor": 0.3}      suma un offset constante
  {"tipo": "escala",  "valor": 0.8}      error de ganancia
  {"tipo": "atascado"}                   repite el ultimo valor
  {"tipo": "ruido",   "valor": 0.5}      ruido extra (desviacion)
  {"tipo": "perdida", "valor": 0.2}      probabilidad de devolver NaN (dato perdido)
  {"tipo": "deriva",  "valor": 0.01}     deriva lenta por segundo desde que aparece
"""
from collections import deque

import numpy as np


class Sensor:
    def __init__(self, nombre="sensor", ruido=0.0, offset=0.0, ganancia=1.0,
                 rango=(-np.inf, np.inf), bits=None, retardo=0, deriva=0.0, semilla=None):
        self.nombre, self.ruido, self.offset, self.ganancia = nombre, ruido, offset, ganancia
        self.rango, self.bits, self.deriva = rango, bits, deriva
        self.buffer = deque([np.nan] * retardo + [None], maxlen=retardo + 1) if retardo else None
        self.rng = np.random.default_rng(semilla)
        self.falla, self.t_falla, self.ultimo = None, None, 0.0

    def medir(self, valor, t=0.0):
        v = self.ganancia * valor + self.offset + self.deriva * t
        f = self.falla
        if f:
            if self.t_falla is None:
                self.t_falla = t
            tipo = f["tipo"]
            if tipo == "atascado":
                return self.ultimo
            if tipo == "sesgo":
                v += f["valor"]
            elif tipo == "escala":
                v *= f["valor"]
            elif tipo == "deriva":
                v += f["valor"] * (t - self.t_falla)
            elif tipo == "ruido":
                v += self.rng.normal(0, f["valor"])
            elif tipo == "perdida" and self.rng.random() < f["valor"]:
                return np.nan
        v += self.rng.normal(0, self.ruido) if self.ruido else 0.0
        lo, hi = self.rango
        v = float(np.clip(v, lo, hi))
        if self.bits and np.isfinite(lo) and np.isfinite(hi):
            paso = (hi - lo) / (2 ** self.bits - 1)
            v = lo + round((v - lo) / paso) * paso
        if self.buffer is not None:
            self.buffer.append(v)
            v = self.buffer[0]
        self.ultimo = v
        return v


class Encoder:
    """Encoder incremental en cuadratura: cpr = cuentas por vuelta (ppr x 4)."""

    def __init__(self, ppr=100, semilla=None):
        self.cpr = 4 * ppr
        self.rng = np.random.default_rng(semilla)
        self.falla = None            # {"tipo": "pulsos_perdidos", "valor": prob}
        self.perdidas = 0
        self.c_prev = 0

    def cuentas(self, theta):
        c = int(np.floor(theta / (2 * np.pi) * self.cpr))
        if self.falla and self.falla["tipo"] == "pulsos_perdidos":
            nuevas = c - self.c_prev - self.perdidas
            self.perdidas += self.rng.binomial(max(nuevas, 0), self.falla["valor"])
        return c - self.perdidas

    def velocidad(self, theta, dt):
        """Metodo de conteo: delta de cuentas en el periodo de muestreo."""
        c = self.cuentas(theta)
        w = (c - self.c_prev) * 2 * np.pi / self.cpr / dt
        self.c_prev = c + self.perdidas
        return w


class PuenteH:
    """Driver de motor con PWM: cuantizacion, zona muerta, limite de cambio y fallas."""

    def __init__(self, V_bus=12.0, zona_muerta=0.03, bits_pwm=10, max_cambio_por_s=None):
        self.V_bus, self.zm, self.bits = V_bus, zona_muerta, bits_pwm
        self.max_cambio = max_cambio_por_s
        self.falla = None            # {"tipo": "ganancia", "valor": 0.7} o {"tipo": "abierto"}
        self.d_prev = 0.0

    def aplicar(self, duty, dt=1e-3):
        d = float(np.clip(duty, -1, 1))
        n = 2 ** self.bits - 1
        d = round(d * n) / n
        if self.max_cambio:
            paso = self.max_cambio * dt
            d = float(np.clip(d, self.d_prev - paso, self.d_prev + paso))
        self.d_prev = d
        efectivo = 0.0 if abs(d) < self.zm else np.sign(d) * (abs(d) - self.zm) / (1 - self.zm)
        V = efectivo * self.V_bus
        if self.falla:
            if self.falla["tipo"] == "ganancia":
                V *= self.falla["valor"]
            elif self.falla["tipo"] == "abierto":
                V = 0.0
        return V
