"""
SIMULADOR INTERACTIVO EN VIVO: motor + MCU + gemelo digital
===========================================================
Una ventana con el motor funcionando en tiempo real y su gemelo al lado.

  Deslizadores : velocidad de referencia y carga de la bomba
  Botones      : inyectar fallas (rodamiento, sensor de corriente, puente H) y reparar

El gemelo (Kalman + residuos) muestra lo que estima y diagnostica en vivo:
  - "Falla MECANICA"  si el par que estima no cuadra con la carga declarada
  - "Falla ELECTRICA / SENSOR" si no se cumple la ley de Kirchhoff (ARR)

Ejecutar:   python simulador_en_vivo.py
Prueba sin ventanas (guarda una captura):  python simulador_en_vivo.py --prueba
"""
import sys
from collections import deque
from pathlib import Path

import numpy as np
import matplotlib

if "--prueba" in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, Slider

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gemelo.estimacion import FiltroKalman, discretizar
from gemelo.graficos import COL
from gemelo.motor import MotorDC, ParamsMotor, matrices_lineales
from gemelo.sensores import Encoder, PuenteH, Sensor

DT = 1e-3
PASOS_POR_CUADRO = 40           # 40 ms de simulacion por cuadro (~tiempo real a 25 fps)
VENTANA = 6.0                   # segundos visibles
P = ParamsMotor()


class Sistema:
    def __init__(self):
        self.motor = MotorDC(P.copia())
        self.s_i = Sensor(ruido=0.02, semilla=1)
        self.enc = Encoder(ppr=1000, semilla=2)
        self.puente = PuenteH(zona_muerta=0.0, bits_pwm=10)
        A, B = matrices_lineales(P)
        F, Bd = discretizar(A, B, DT)
        H = np.array([[1.0, 0, 0], [0, 1.0, 0]])
        self.kf = FiltroKalman(F, Bd, H, np.diag([1e-4, 0.05, 2e-6]), np.diag([0.02 ** 2, 0.6]),
                               np.zeros(3), np.eye(3) * 0.1)
        self.ref, self.carga = 120.0, 0.02
        self.integ, self.t = 0.0, 0.0
        self.r_e = 0.0
        self.tl_f = 0.0
        self.i_prev = 0.0
        n = int(VENTANA / (DT * 10))
        self.hist = {k: deque(maxlen=n) for k in ["t", "ref", "w", "w_g", "i_m", "i_g", "TL", "TL_g", "r_e", "r_m"]}
        self.estado = "OK"
        self.cont_e = self.cont_m = 0          # persistencia: evita alarmas por transitorios

    def reparar(self):
        self.motor.p.b = P.b
        self.s_i.falla = None
        self.puente.falla = None

    def paso(self):
        m = self.motor
        w_m = self.enc.velocidad(m.theta, DT)
        i_m = self.s_i.medir(m.i, self.t)
        e = self.ref - w_m
        self.integ = np.clip(self.integ + 0.08 * e * DT, 0, 1)
        duty = np.clip(0.004 * e + self.integ, 0, 1)
        V_cmd = duty * P.V_bus
        TL = self.carga + 0.0 * m.w
        m.paso(self.puente.aplicar(duty, DT), DT, TL); m.T = 25.0
        # ---- gemelo
        self.kf.predecir(V_cmd)
        k = int(round(self.t / DT))
        if k % 10 == 0:
            self.kf.corregir([i_m, w_m])
            di = (i_m - self.i_prev) / (10 * DT); self.i_prev = i_m
            arr = V_cmd - P.R0 * i_m - P.Kt * w_m - P.L * di
            self.r_e += 0.05 * (arr - self.r_e)
            self.tl_f += 0.05 * (self.kf.x[2] - self.tl_f)
            r_m = self.tl_f - (self.carga + P.Tc)            # par no explicado por la carga declarada
            for key, v in zip(self.hist, [self.t, self.ref, m.w, self.kf.x[1], i_m, self.kf.x[0],
                                          TL, self.kf.x[2], self.r_e, r_m]):
                self.hist[key].append(v)
            self.cont_e = self.cont_e + 1 if abs(self.r_e) > 0.35 else 0
            self.cont_m = self.cont_m + 1 if abs(r_m) > 0.006 else 0
            electrica = self.cont_e >= 20                      # 0.2 s seguidos
            mecanica = self.cont_m >= 50 and not electrica     # 0.5 s seguidos
            self.estado = ("ALERTA: falla ELECTRICA o de SENSOR (no se cumple Kirchhoff)" if electrica else
                           "ALERTA: falla MECANICA (friccion o carga anormal)" if mecanica else "OK")
        self.t += DT


def construir_ventana(sis):
    fig = plt.figure(figsize=(13, 8.5))
    fig.subplots_adjust(left=0.07, right=0.97, top=0.93, bottom=0.27, hspace=0.45)
    gs = fig.add_gridspec(2, 2)
    ax_w, ax_i = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1])
    ax_tl, ax_r = fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1])
    L = {}
    L["ref"], = ax_w.plot([], [], "k:", label="referencia")
    L["w"], = ax_w.plot([], [], color=COL["fisico"], lw=2, label="real")
    L["w_g"], = ax_w.plot([], [], "--", color=COL["gemelo"], label="gemelo")
    ax_w.set_ylim(0, 230); ax_w.set_title("Velocidad (rad/s)"); ax_w.legend(loc="lower left", fontsize=8)
    L["i_m"], = ax_i.plot([], [], color=COL["medido"], lw=0.8, label="medida")
    L["i_g"], = ax_i.plot([], [], color=COL["gemelo"], label="gemelo")
    ax_i.set_ylim(-0.2, 3.5); ax_i.set_title("Corriente (A)"); ax_i.legend(loc="lower left", fontsize=8)
    L["TL"], = ax_tl.plot([], [], color=COL["fisico"], lw=2, label="carga declarada")
    L["TL_g"], = ax_tl.plot([], [], color=COL["gemelo"], label="par estimado por el gemelo")
    ax_tl.set_ylim(-0.01, 0.12); ax_tl.set_title("Par de carga (N*m) - sensor virtual")
    ax_tl.legend(loc="upper left", fontsize=8)
    L["r_e"], = ax_r.plot([], [], color=COL["alerta"], label="residuo electrico ARR (V)")
    L["r_m"], = ax_r.plot([], [], color=COL["pred"], label="residuo mecanico x50 (N*m)")
    for y in (0.35, -0.35):
        ax_r.axhline(y, color=COL["falla"], ls="--", lw=0.8)
    ax_r.set_ylim(-1.5, 3); ax_r.set_title("Residuos del gemelo"); ax_r.legend(loc="upper left", fontsize=8)
    estado = fig.text(0.5, 0.965, "", ha="center", fontsize=13, fontweight="bold")

    s_ref = Slider(fig.add_axes([0.12, 0.17, 0.33, 0.03]), "Velocidad ref", 40, 200, valinit=sis.ref)
    s_car = Slider(fig.add_axes([0.62, 0.17, 0.33, 0.03]), "Carga (N*m)", 0.0, 0.06, valinit=sis.carga)
    s_ref.on_changed(lambda v: setattr(sis, "ref", v))
    s_car.on_changed(lambda v: setattr(sis, "carga", v))
    botones = [("Falla rodamiento", lambda e: setattr(sis.motor.p, "b", P.b * 25)),
               ("Falla sensor i", lambda e: setattr(sis.s_i, "falla", {"tipo": "sesgo", "valor": 0.4})),
               ("Falla puente H", lambda e: setattr(sis.puente, "falla", {"tipo": "ganancia", "valor": 0.8})),
               ("Reparar todo", lambda e: sis.reparar())]
    widgets = [s_ref, s_car]
    for k, (txt, fn) in enumerate(botones):
        b = Button(fig.add_axes([0.07 + k * 0.23, 0.05, 0.2, 0.06]), txt,
                   color="#fde0dd" if "Falla" in txt else "#e5f5e0")
        b.on_clicked(fn)
        widgets.append(b)

    def actualizar(_):
        for _ in range(PASOS_POR_CUADRO):
            sis.paso()
        h = sis.hist
        t = np.array(h["t"])
        for k in ["ref", "w", "w_g", "i_m", "i_g", "TL", "TL_g", "r_e"]:
            L[k].set_data(t, np.array(h[k]))
        L["r_m"].set_data(t, np.array(h["r_m"]) * 50)
        L["TL"].set_data(t, np.array(h["TL"]) + P.Tc)
        for ax in (ax_w, ax_i, ax_tl, ax_r):
            ax.set_xlim(max(0, sis.t - VENTANA), max(VENTANA, sis.t))
        estado.set_text(sis.estado)
        estado.set_color(COL["ok"] if sis.estado == "OK" else COL["falla"])
        return list(L.values()) + [estado]

    return fig, actualizar, widgets


def main():
    sis = Sistema()
    fig, actualizar, widgets = construir_ventana(sis)
    if "--prueba" in sys.argv:
        salida = Path(__file__).resolve().parent / "simulador_captura.png"
        guion = {75: lambda: setattr(sis.motor.p, "b", P.b * 25), 150: sis.reparar,
                 200: lambda: setattr(sis.s_i, "falla", {"tipo": "sesgo", "valor": 0.4})}
        estados = []
        for cuadro in range(260):
            if cuadro in guion:
                guion[cuadro]()
            actualizar(cuadro)
            if cuadro in (70, 145, 195, 259):
                estados.append((round(sis.t, 1), sis.estado))
        fig.savefig(salida, dpi=90)
        print("Prueba del simulador (sin ventanas):")
        for t, e in estados:
            print(f"  t = {t:5.1f} s -> {e}")
        print(f"  Captura: {salida}")
        return
    anim = FuncAnimation(fig, actualizar, interval=40, blit=False, cache_frame_data=False)
    plt.show()
    return anim


if __name__ == "__main__":
    main()
