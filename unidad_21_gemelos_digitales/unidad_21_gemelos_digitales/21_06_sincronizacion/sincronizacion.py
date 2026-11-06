"""
21.6 Sincronizacion entre sistema virtual y fisico
==================================================
El motor real no es igual al modelo (parametros distintos, carga desconocida).
Tres estrategias para que el gemelo siga al real:

  A) Lazo abierto       : el gemelo recibe la misma tension y simula. Se desvia.
  B) Kalman [i, w]      : corrige el estado con las mediciones.
  C) Kalman [i, w, TL]  : ademas estima el par de carga -> "sensor virtual".

Tambien se estudia:
  - cada cuanto hay que sincronizar (periodo de actualizacion)
  - que pasa si se corta la comunicacion (la incertidumbre crece y se recupera)

Genera: 1_estrategias.png, 2_sensor_virtual.png, 3_periodo_sync.png, 4_corte_comunicacion.png
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.estimacion import FiltroKalman, discretizar
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.motor import MotorDC, ParamsMotor, matrices_lineales
from gemelo.sensores import Encoder, Sensor

SALIDA = carpeta_resultados(__file__)
DT, T_TOTAL = 1e-3, 4.0
P_NOM = ParamsMotor()
P_REAL = P_NOM.copia(R0=2.2, Kt=0.048, b=4e-5, Tc=0.003, J=1.1e-4)


def carga(t):
    return 0.0 if t < 1 else (0.03 if t < 2 else (0.06 if t < 3 else 0.01))


def tension(t):
    return 9 + 3 * np.sign(np.sin(2 * np.pi * 0.7 * t)) + 1.5 * np.sin(2 * np.pi * 3 * t)


def datos_fisicos(semilla=0):
    motor = MotorDC(P_REAL)
    s_i, enc = Sensor(ruido=0.03, semilla=semilla), Encoder(ppr=1000, semilla=semilla)
    n = int(T_TOTAL / DT)
    r = {k: np.zeros(n) for k in ["t", "V", "i", "w", "TL", "i_m", "w_m"]}
    for k in range(n):
        t = k * DT
        V = tension(t)
        motor.paso(V, DT, carga(t))
        r["t"][k], r["V"][k], r["i"][k], r["w"][k], r["TL"][k] = t, V, motor.i, motor.w, carga(t)
        r["i_m"][k], r["w_m"][k] = s_i.medir(motor.i), enc.velocidad(motor.theta, DT)
    return r


def crear_kf(con_carga):
    A, B = matrices_lineales(P_NOM, con_carga=con_carga)
    F, Bd = discretizar(A, B, DT)
    n = F.shape[0]
    H = np.zeros((2, n)); H[0, 0] = H[1, 1] = 1
    # B debe "absorber" la carga desconocida con ruido de proceso grande en w;
    # C la explica con su propio estado, asi que puede confiar mas en el modelo.
    Q = np.diag([1e-4, 0.05, 2e-6]) if con_carga else np.diag([1e-4, 0.5])
    R = np.diag([0.03 ** 2, (2 * np.pi / 4000 / DT) ** 2 / 12 + 0.5])
    return FiltroKalman(F, Bd, H, Q, R, np.zeros(n), np.eye(n) * 0.1)


def correr_kf(d, con_carga, cada=1, corte=None):
    kf = crear_kf(con_carga)
    n = len(d["t"])
    X, S = np.zeros((n, kf.x.size)), np.zeros((n, kf.x.size))
    for k in range(n):
        kf.predecir(d["V"][k])
        sin_datos = corte is not None and corte[0] <= d["t"][k] < corte[1]
        if k % cada == 0 and not sin_datos:
            kf.corregir([d["i_m"][k], d["w_m"][k]])
        X[k], S[k] = kf.x, np.sqrt(np.diag(kf.P))
    return X, S


def lazo_abierto(d):
    m = MotorDC(P_NOM)
    W, I = [], []
    for V in d["V"]:
        m.paso(V, DT, 0.0); m.T = 25.0
        W.append(m.w); I.append(m.i)
    return np.array(I), np.array(W)


def rms(a):
    return np.sqrt(np.mean(a ** 2))


def main():
    titulo("21.6 Sincronizacion entre sistema virtual y fisico")
    d = datos_fisicos()
    t = d["t"]
    i_la, w_la = lazo_abierto(d)
    SYNC = 20                                          # telemetria a 50 Hz (cada 20 ms)
    Xb, _ = correr_kf(d, False, cada=SYNC)
    Xc, Sc = correr_kf(d, True, cada=SYNC)

    # --- 1. estrategias
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    ax1.plot(t, d["w_m"], color=COL["medido"], lw=0.5, label="medicion encoder")
    ax1.plot(t, d["w"], color=COL["fisico"], lw=2, label="real")
    ax1.plot(t, w_la, color=COL["modelo"], label=f"A) lazo abierto (RMS {rms(w_la - d['w']):.1f})")
    ax1.plot(t, Xb[:, 1], color=COL["alerta"], label=f"B) Kalman [i,w] (RMS {rms(Xb[:, 1] - d['w']):.1f})")
    ax1.plot(t, Xc[:, 1], color=COL["gemelo"], label=f"C) Kalman [i,w,TL] (RMS {rms(Xc[:, 1] - d['w']):.1f})")
    ax1.set_ylabel("Velocidad (rad/s)"); ax1.legend(fontsize=8, ncol=2)
    ax2.plot(t, w_la - d["w"], color=COL["modelo"], label="A")
    ax2.plot(t, Xb[:, 1] - d["w"], color=COL["alerta"], label="B")
    ax2.plot(t, Xc[:, 1] - d["w"], color=COL["gemelo"], label="C")
    for tc in (1, 2, 3):
        ax2.axvline(tc, color="gray", alpha=0.4)
    ax2.set_ylabel("Error del gemelo (rad/s)"); ax2.set_xlabel("Tiempo (s)  [lineas: cambia la carga]")
    ax2.legend()
    fig.suptitle("1. Sincronizando cada 20 ms: sin sincronizar el gemelo se aleja; "
                 "con el estado de carga predice bien entre mediciones")
    fig.tight_layout(); guardar(fig, SALIDA, "1_estrategias.png")
    for n, w in [("A lazo abierto", w_la), ("B Kalman [i,w]", Xb[:, 1]), ("C Kalman [i,w,TL]", Xc[:, 1])]:
        print(f"  {n:20s}: RMS velocidad = {rms(w - d['w']):6.2f} rad/s")

    # --- 2. sensor virtual de par de carga
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, d["TL"], color=COL["fisico"], lw=2, label="par de carga real (no se mide)")
    ax.plot(t, Xc[:, 2], color=COL["gemelo"], label="estimado por el gemelo")
    ax.fill_between(t, Xc[:, 2] - 2 * Sc[:, 2], Xc[:, 2] + 2 * Sc[:, 2], color=COL["gemelo"], alpha=0.2,
                    label="+-2 sigma")
    ax.set_ylabel("N*m"); ax.set_xlabel("Tiempo (s)"); ax.legend()
    ax.set_title("2. Sensor virtual: el gemelo 've' una magnitud sin sensor fisico "
                 "(incluye friccion no modelada: offset ~ Tc)")
    fig.tight_layout(); guardar(fig, SALIDA, "2_sensor_virtual.png")

    # --- 3. periodo de sincronizacion
    periodos = [1, 2, 5, 10, 20, 50, 100, 200]
    err_b = [rms(correr_kf(d, False, cada=p)[0][:, 1] - d["w"]) for p in periodos]
    err = [rms(correr_kf(d, True, cada=p)[0][:, 1] - d["w"]) for p in periodos]
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.semilogx(np.array(periodos) * DT * 1000, err_b, "s-", color=COL["alerta"], label="B Kalman [i,w]")
    ax.semilogx(np.array(periodos) * DT * 1000, err, "o-", color=COL["gemelo"], label="C Kalman [i,w,TL]")
    ax.axhline(rms(w_la - d["w"]), color=COL["modelo"], ls="--", label="sin sincronizar")
    ax.set_xlabel("Periodo de sincronizacion (ms)"); ax.set_ylabel("Error RMS (rad/s)")
    ax.set_title("3. Cuanto mas lenta la sincronizacion, mas se aleja el gemelo"); ax.legend()
    fig.tight_layout(); guardar(fig, SALIDA, "3_periodo_sync.png")
    for p, eb, e in zip(periodos, err_b, err):
        print(f"  sincronizar cada {p * DT * 1000:5.0f} ms -> RMS  B: {eb:6.2f}   C: {e:6.2f} rad/s")

    # --- 4. corte de comunicacion
    Xk, Sk = correr_kf(d, True, cada=SYNC, corte=(2.2, 2.8))
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, d["w"], color=COL["fisico"], lw=2, label="real")
    ax.plot(t, Xk[:, 1], color=COL["gemelo"], label="gemelo")
    ax.fill_between(t, Xk[:, 1] - 2 * Sk[:, 1], Xk[:, 1] + 2 * Sk[:, 1], color=COL["gemelo"], alpha=0.25,
                    label="incertidumbre +-2 sigma")
    ax.axvspan(2.2, 2.8, color=COL["falla"], alpha=0.1, label="sin datos (corte de red)")
    ax.set_xlim(1.8, 3.4); ax.set_xlabel("Tiempo (s)"); ax.set_ylabel("rad/s"); ax.legend(fontsize=8)
    ax.set_title("4. Durante el corte el gemelo sigue prediciendo y SABE que su incertidumbre crece; luego se resincroniza")
    fig.tight_layout(); guardar(fig, SALIDA, "4_corte_comunicacion.png")
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
