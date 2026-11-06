"""
Robot movil de traccion diferencial.

  v  = (r_der*w_der + r_izq*w_izq) / 2
  om = (r_der*w_der - r_izq*w_izq) / base
  x' = v cos(th),  y' = v sin(th),  th' = om
Cada rueda tiene un motor modelado como sistema de primer orden.
El robot "fisico" tiene radios distintos, base distinta y deslizamiento:
justo lo que hace que la odometria del gemelo se desvie.
"""
from dataclasses import dataclass, replace

import numpy as np


@dataclass
class ParamsRobot:
    r_izq: float = 0.033      # m
    r_der: float = 0.033      # m
    base: float = 0.150       # m
    tau: float = 0.08         # s, constante de tiempo de cada motor
    w_max: float = 25.0       # rad/s
    desliz: float = 0.0       # desviacion del ruido de deslizamiento (fraccion)

    def copia(self, **c):
        return replace(self, **c)


class RobotDiferencial:
    def __init__(self, p=None, x=0.0, y=0.0, th=0.0, semilla=None):
        self.p = p or ParamsRobot()
        self.x, self.y, self.th = x, y, th
        self.wi = self.wd = 0.0
        self.ang_i = self.ang_d = 0.0       # angulo acumulado de cada rueda (encoder)
        self.rng = np.random.default_rng(semilla)

    def paso(self, cmd_i, cmd_d, dt):
        p = self.p
        cmd_i = np.clip(cmd_i, -p.w_max, p.w_max)
        cmd_d = np.clip(cmd_d, -p.w_max, p.w_max)
        self.wi += (cmd_i - self.wi) * dt / p.tau
        self.wd += (cmd_d - self.wd) * dt / p.tau
        self.ang_i += self.wi * dt
        self.ang_d += self.wd * dt
        s_i = 1 - abs(self.rng.normal(0, p.desliz)) if p.desliz else 1.0
        s_d = 1 - abs(self.rng.normal(0, p.desliz)) if p.desliz else 1.0
        vi, vd = p.r_izq * self.wi * s_i, p.r_der * self.wd * s_d
        v, om = (vd + vi) / 2, (vd - vi) / p.base
        self.x += v * np.cos(self.th) * dt
        self.y += v * np.sin(self.th) * dt
        self.th = (self.th + om * dt + np.pi) % (2 * np.pi) - np.pi
        return v, om

    @property
    def pose(self):
        return np.array([self.x, self.y, self.th])


def ir_a_punto(pose, objetivo, v_max=0.25, k_ang=3.0, p=None):
    """Controlador simple: girar hacia el objetivo y avanzar. Devuelve (w_izq, w_der)."""
    p = p or ParamsRobot()
    dx, dy = objetivo[0] - pose[0], objetivo[1] - pose[1]
    dist = np.hypot(dx, dy)
    err = (np.arctan2(dy, dx) - pose[2] + np.pi) % (2 * np.pi) - np.pi
    v = min(v_max, 1.5 * dist) * max(0.0, np.cos(err)) ** 2
    om = k_ang * err
    r = (p.r_izq + p.r_der) / 2
    return (v - om * p.base / 2) / r, (v + om * p.base / 2) / r, dist


def odometria(pose, d_ang_i, d_ang_d, p):
    """Actualiza la pose con los incrementos de angulo de las ruedas (dead reckoning)."""
    di, dd = p.r_izq * d_ang_i, p.r_der * d_ang_d
    ds, dth = (dd + di) / 2, (dd - di) / p.base
    th_m = pose[2] + dth / 2
    nueva = pose + np.array([ds * np.cos(th_m), ds * np.sin(th_m), dth])
    nueva[2] = (nueva[2] + np.pi) % (2 * np.pi) - np.pi
    return nueva, ds, dth
