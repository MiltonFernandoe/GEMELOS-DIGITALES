"""
Motor DC de iman permanente: modelo electrico + mecanico + termico.

  L di/dt = V - R(T) i - Ke w                 (electrico)
  J dw/dt = Kt i - b w - Tc*tanh(w/w0) - TL   (mecanico, con friccion de Coulomb)
  Cth dT/dt = R(T) i^2 - (T - T_amb)/Rth      (termico del bobinado)
  R(T) = R0 (1 + alfa (T - 25))               (el cobre sube ~0.39 %/C)

En un motor de iman permanente Ke = Kt (en unidades SI).
Valores por defecto: motor de 12 V tipo robotica (~2300 rpm en vacio).
"""
from dataclasses import dataclass, replace

import numpy as np


@dataclass
class ParamsMotor:
    R0: float = 2.0        # ohm a 25 C
    L: float = 5e-3        # H
    Kt: float = 0.05       # N*m/A  (= Ke en V*s/rad)
    J: float = 1e-4        # kg*m^2
    b: float = 2e-5        # N*m*s/rad  friccion viscosa (rodamientos)
    Tc: float = 0.002      # N*m  friccion de Coulomb
    alfa: float = 0.0039   # 1/C  coeficiente termico del cobre
    Rth: float = 6.0       # C/W  resistencia termica bobinado->ambiente
    Cth: float = 10.0      # J/C  capacidad termica del bobinado
    T_amb: float = 25.0    # C
    V_bus: float = 12.0    # V
    T_max: float = 130.0   # C  aislamiento clase B

    def copia(self, **cambios):
        return replace(self, **cambios)


class MotorDC:
    def __init__(self, p=None):
        self.p = p or ParamsMotor()
        self.reset()

    def reset(self, T0=None):
        self.i = self.w = self.theta = self.t = 0.0
        self.T = self.p.T_amb if T0 is None else T0

    @property
    def R(self):
        return self.p.R0 * (1 + self.p.alfa * (self.T - 25.0))

    def _f(self, i, w, V, TL):
        p = self.p
        di = (V - self.R * i - p.Kt * w) / p.L
        dw = (p.Kt * i - p.b * w - p.Tc * np.tanh(w / 0.5) - TL) / p.J
        return di, dw

    def paso(self, V, dt, T_carga=0.0):
        """Integra dt segundos con Runge-Kutta 4 (dt <= 1 ms recomendado)."""
        i, w = self.i, self.w
        k1 = self._f(i, w, V, T_carga)
        k2 = self._f(i + dt / 2 * k1[0], w + dt / 2 * k1[1], V, T_carga)
        k3 = self._f(i + dt / 2 * k2[0], w + dt / 2 * k2[1], V, T_carga)
        k4 = self._f(i + dt * k3[0], w + dt * k3[1], V, T_carga)
        self.i = i + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        self.w = w + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        self.theta += self.w * dt
        self._termico(dt)
        self.t += dt
        return self.w

    def regimen(self, V, T_carga=0.0, k_cuad=0.0):
        """Velocidad y corriente en regimen permanente (para simulaciones largas).
        Carga = T_carga + k_cuad * w^2  (k_cuad > 0: bomba o ventilador)."""
        p, R = self.p, self.R
        D = p.b + p.Kt ** 2 / R
        N = p.Kt * V / R - T_carga - p.Tc
        if N <= 0:
            w = 0.0
        elif k_cuad > 0:
            w = (-D + np.sqrt(D * D + 4 * k_cuad * N)) / (2 * k_cuad)
        else:
            w = N / D
        return (V - p.Kt * w) / R, w

    def paso_lento(self, V, dt, T_carga=0.0, k_cuad=0.0):
        """Para horizontes de minutos/horas: electrico y mecanico en regimen, termico dinamico."""
        self.i, self.w = self.regimen(V, T_carga, k_cuad)
        self.theta += self.w * dt
        self._termico(dt)
        self.t += dt
        return self.w

    def _termico(self, dt):
        p = self.p
        self.T += dt * (self.R * self.i ** 2 - (self.T - p.T_amb) / p.Rth) / p.Cth

    def potencia(self, V):
        return V * self.i, self.p.Kt * self.i * self.w   # (electrica, mecanica)


def matrices_lineales(p, T=25.0, con_carga=True):
    """Modelo lineal para el filtro de Kalman.
    Estado [i, w] o [i, w, T_carga] (la carga se modela como paseo aleatorio)."""
    R = p.R0 * (1 + p.alfa * (T - 25.0))
    if con_carga:
        A = np.array([[-R / p.L, -p.Kt / p.L, 0.0],
                      [p.Kt / p.J, -p.b / p.J, -1.0 / p.J],
                      [0.0, 0.0, 0.0]])
        B = np.array([[1.0 / p.L], [0.0], [0.0]])
    else:
        A = np.array([[-R / p.L, -p.Kt / p.L], [p.Kt / p.J, -p.b / p.J]])
        B = np.array([[1.0 / p.L], [0.0]])
    return A, B
