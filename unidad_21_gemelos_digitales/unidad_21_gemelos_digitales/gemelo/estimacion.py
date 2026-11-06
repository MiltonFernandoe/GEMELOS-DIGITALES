"""Estimadores usados para sincronizar el gemelo con el sistema fisico."""
import numpy as np


def expm(M):
    """Exponencial de matriz (escalado y cuadrado + serie de Taylor)."""
    norma = np.linalg.norm(M, 1)
    n = max(0, int(np.ceil(np.log2(norma))) + 1) if norma > 0 else 0
    A = M / 2 ** n
    E = np.eye(len(M))
    T = np.eye(len(M))
    for k in range(1, 16):
        T = T @ A / k
        E = E + T
    for _ in range(n):
        E = E @ E
    return E


def discretizar(A, B, dt):
    """Modelo continuo x' = Ax + Bu  ->  discreto x[k+1] = Ad x[k] + Bd u[k] (ZOH exacto)."""
    n, m = B.shape
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A * dt
    M[:n, n:] = B * dt
    E = expm(M)
    return E[:n, :n], E[:n, n:]


class FiltroKalman:
    """Filtro de Kalman lineal:  x[k+1] = F x + B u + w,   z = H x + v."""

    def __init__(self, F, B, H, Q, R, x0, P0):
        self.F, self.B, self.H = F, B, H
        self.Q, self.R = Q, R
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)

    def predecir(self, u):
        self.x = self.F @ self.x + self.B @ np.atleast_1d(u)
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x

    def corregir(self, z, H=None, R=None):
        H = self.H if H is None else H
        R = self.R if R is None else R
        z = np.atleast_1d(z)
        innov = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.P = (np.eye(len(self.x)) - K @ H) @ self.P
        return innov, S


class EKF:
    """Filtro de Kalman extendido para modelos no lineales."""

    def __init__(self, f, F_jac, Q, x0, P0):
        self.f, self.F_jac, self.Q = f, F_jac, Q
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)

    def predecir(self, u, dt):
        F = self.F_jac(self.x, u, dt)
        self.x = self.f(self.x, u, dt)
        self.P = F @ self.P @ F.T + self.Q
        return self.x

    def corregir(self, z, h, H, R, angulos=()):
        innov = np.atleast_1d(z) - h(self.x)
        for k in angulos:                                   # envolver angulos a [-pi, pi]
            innov[k] = (innov[k] + np.pi) % (2 * np.pi) - np.pi
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.P = (np.eye(len(self.x)) - K @ H) @ self.P
        return innov


class RLS:
    """Minimos cuadrados recursivos con factor de olvido: y = theta^T x."""

    def __init__(self, theta0, p0=100.0, lam=0.999):
        self.theta = np.array(theta0, dtype=float)
        self.P = np.eye(len(theta0)) * p0
        self.lam = lam

    def actualizar(self, x, y):
        x = np.asarray(x, dtype=float)
        Px = self.P @ x
        k = Px / (self.lam + x @ Px)
        e = y - x @ self.theta
        self.theta = self.theta + k * e
        self.P = (self.P - np.outer(k, Px)) / self.lam
        return e
