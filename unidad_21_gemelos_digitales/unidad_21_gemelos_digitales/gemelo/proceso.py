"""
Planta industrial: dos tanques en cascada con bomba, valvula intermedia y salida.

  A1 dh1/dt = k_b * u  - c12 * sqrt(h1)               - fuga * sqrt(h1)
  A2 dh2/dt = c12 * sqrt(h1) - c2 * a_sal * sqrt(h2)
u: comando de la bomba (0..1), a_sal: apertura de la valvula de consumo (0..1).
Es el ejemplo clasico de control de procesos (Torricelli).
"""
from dataclasses import dataclass, replace

import numpy as np


@dataclass
class ParamsTanques:
    A1: float = 3.0        # m^2
    A2: float = 3.0        # m^2
    k_b: float = 0.012     # m^3/s con la bomba al 100 %
    c12: float = 0.008     # m^2.5/s
    c2: float = 0.009
    h_max: float = 2.0     # m (rebalse)

    def copia(self, **c):
        return replace(self, **c)


class Tanques:
    def __init__(self, p=None, h1=0.5, h2=0.5):
        self.p = p or ParamsTanques()
        self.h1, self.h2 = h1, h2
        self.fuga = 0.0                # coeficiente de fuga en el tanque 1 (falla)
        self.bomba_ok = True           # falla de bomba

    def caudales(self, u, a_sal):
        p = self.p
        q_in = p.k_b * np.clip(u, 0, 1) * (1.0 if self.bomba_ok else 0.0)
        q12 = p.c12 * np.sqrt(max(self.h1, 0))
        q_fuga = self.fuga * np.sqrt(max(self.h1, 0))
        q_out = p.c2 * np.clip(a_sal, 0, 1) * np.sqrt(max(self.h2, 0))
        return q_in, q12, q_fuga, q_out

    def paso(self, u, a_sal, dt):
        q_in, q12, q_fuga, q_out = self.caudales(u, a_sal)
        p = self.p
        self.h1 = float(np.clip(self.h1 + dt * (q_in - q12 - q_fuga) / p.A1, 0, p.h_max))
        self.h2 = float(np.clip(self.h2 + dt * (q12 - q_out) / p.A2, 0, p.h_max))
        return self.h1, self.h2
