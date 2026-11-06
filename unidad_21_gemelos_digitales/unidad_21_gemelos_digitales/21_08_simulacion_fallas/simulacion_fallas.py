"""
21.8 Simulacion de fallas
=========================
Se inyectan 7 fallas en el motor (sensores, actuador, mecanica, electrica)
mientras trabaja en lazo cerrado. El gemelo genera RESIDUOS:

  r_i = i_medida - i_gemelo         (corriente)
  r_w = w_medida - w_gemelo         (velocidad)
  r_e = V_mando - R*i_med - Ke*w_med  (relacion de redundancia analitica, ARR
                                       electrica: solo usa mediciones + la ley de Kirchhoff)

Cada falla deja una FIRMA en los residuos -> deteccion y AISLAMIENTO.
A los 3 s cambia la consigna (150 -> 120 rad/s): sin excitacion, un sensor
atascado en regimen permanente es indetectable.

Genera: 1_residuos.png, 2_firmas.png, 3_tiempos_deteccion.png
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.motor import MotorDC, ParamsMotor
from gemelo.sensores import Encoder, PuenteH, Sensor

SALIDA = carpeta_resultados(__file__)
DT, T_TOTAL, T_FALLA = 1e-3, 4.0, 2.0
CARGA = 0.02
P = ParamsMotor()

FALLAS = {
    "sin falla": {},
    "sensor corriente: sesgo +0.3 A": {"sensor_i": {"tipo": "sesgo", "valor": 0.3}},
    "sensor corriente: atascado": {"sensor_i": {"tipo": "atascado"}},
    "encoder: pierde 15 % pulsos": {"encoder": {"tipo": "pulsos_perdidos", "valor": 0.15}},
    "puente H: ganancia 80 %": {"puente": {"tipo": "ganancia", "valor": 0.8}},
    "rodamiento: friccion x15": {"param": {"b": P.b * 15}},
    "bobinado: R +30 %": {"param": {"R0": P.R0 * 1.3}},
    "sobrecarga: +0.03 N*m": {"carga": 0.03},
}


def correr(nombre, falla, semilla=0):
    motor = MotorDC(P.copia())
    gemelo = MotorDC(P.copia())
    s_i = Sensor(ruido=0.02, semilla=semilla)
    enc = Encoder(ppr=1000, semilla=semilla)
    puente = PuenteH(zona_muerta=0.0, bits_pwm=12)
    integ, n = 0.0, int(T_TOTAL / DT)
    r = {k: np.zeros(n) for k in ["r_i", "r_w", "r_e", "w"]}
    filt = np.zeros(3)
    for k in range(n):
        t = k * DT
        if t >= T_FALLA and falla:
            s_i.falla = falla.get("sensor_i")
            enc.falla = falla.get("encoder")
            puente.falla = falla.get("puente")
            for clave, val in falla.get("param", {}).items():
                setattr(motor.p, clave, val)
        carga = CARGA + (falla.get("carga", 0.0) if t >= T_FALLA else 0.0)
        w_m = enc.velocidad(motor.theta, DT)
        i_m = s_i.medir(motor.i, t)
        # firmware: PI de velocidad
        e = (150.0 if t < 3.0 else 120.0) - w_m          # cambio de consigna a los 3 s
        integ = np.clip(integ + 0.08 * e * DT, 0, 1)
        duty = np.clip(0.004 * e + integ, 0, 1)
        V_cmd = duty * P.V_bus
        motor.paso(puente.aplicar(duty, DT), DT, carga); motor.T = 25.0
        gemelo.paso(V_cmd, DT, CARGA); gemelo.T = 25.0          # el gemelo cree lo nominal
        crudo = np.array([i_m - gemelo.i, w_m - gemelo.w, V_cmd - P.R0 * i_m - P.Kt * w_m])
        filt += 0.02 * (crudo - filt)                             # filtro pasa bajos (~50 ms)
        r["r_i"][k], r["r_w"][k], r["r_e"][k] = filt
        r["w"][k] = motor.w
    return r


def main():
    titulo("21.8 Simulacion de fallas")
    t = np.arange(int(T_TOTAL / DT)) * DT
    res = {n: correr(n, f) for n, f in FALLAS.items()}
    base = res["sin falla"]
    # Umbrales con operacion normal REPRESENTATIVA (incluye el cambio de consigna).
    # Se usa otra corrida sin falla (otra semilla) para no evaluar con los mismos datos.
    calib = correr("calibracion", {}, semilla=99)
    sano = slice(int(0.5 / DT), int(T_TOTAL / DT))
    umbral = {k: 1.2 * np.max(np.abs(calib[k][sano])) for k in ["r_i", "r_w", "r_e"]}
    print("  Umbrales (1.2 x maximo en operacion normal): " +
          ", ".join(f"{k}={v:.3f}" for k, v in umbral.items()))

    # --- 1. residuos de algunas fallas
    mostrar_f = ["sin falla", "sensor corriente: sesgo +0.3 A", "puente H: ganancia 80 %", "rodamiento: friccion x15"]
    fig, axes = plt.subplots(3, len(mostrar_f), figsize=(16, 8), sharex=True, sharey="row")
    nombres_r = {"r_i": "r_i corriente (A)", "r_w": "r_w velocidad (rad/s)", "r_e": "r_e ARR electrica (V)"}
    for j, f in enumerate(mostrar_f):
        for i, (k, etiqueta) in enumerate(nombres_r.items()):
            ax = axes[i, j]
            ax.plot(t, res[f][k], color=COL["gemelo"], lw=1)
            ax.axhline(umbral[k], color=COL["falla"], ls="--", lw=0.8)
            ax.axhline(-umbral[k], color=COL["falla"], ls="--", lw=0.8)
            ax.axvline(T_FALLA, color="gray", alpha=0.4)
            if j == 0:
                ax.set_ylabel(etiqueta)
        axes[0, j].set_title(f, fontsize=9)
        axes[-1, j].set_xlabel("Tiempo (s)")
    fig.suptitle("1. Residuos: ~0 cuando todo esta bien, salen de la banda cuando aparece una falla (t = 2 s)")
    fig.tight_layout(); guardar(fig, SALIDA, "1_residuos.png")

    # --- 2. matriz de firmas
    nombres = list(FALLAS)
    ventana = slice(int(2.5 / DT), int(T_TOTAL / DT))
    firma = np.array([[np.mean(np.abs(res[f][k][ventana])) / umbral[k] for k in umbral] for f in nombres])
    fig, ax = plt.subplots(figsize=(8, 6))
    im = ax.imshow(np.log10(np.maximum(firma, 0.05)), cmap="RdYlGn_r", vmin=-1, vmax=1.5, aspect="auto")
    for i in range(len(nombres)):
        for j in range(3):
            ax.text(j, i, "1" if firma[i, j] > 1 else "0", ha="center", va="center",
                    fontsize=13, fontweight="bold")
    ax.set_xticks(range(3)); ax.set_xticklabels(["r_i", "r_w", "r_e (ARR)"])
    ax.set_yticks(range(len(nombres))); ax.set_yticklabels(nombres, fontsize=9)
    fig.colorbar(im, label="log10(|residuo| / umbral)")
    ax.set_title("2. Matriz de firmas: patrones distintos = fallas distinguibles\n"
                 "(filas con la misma firma no se pueden aislar con estos 3 residuos)")
    fig.tight_layout(); guardar(fig, SALIDA, "2_firmas.png")
    print(f"\n  {'Falla':34s} r_i r_w r_e   deteccion")

    # --- 3. tiempos de deteccion
    t_det = {}
    for f in nombres[1:]:
        fuera = np.zeros(len(t), bool)
        for k in umbral:
            fuera |= np.abs(res[f][k]) > umbral[k]
        fuera[: int(T_FALLA / DT)] = False
        persist = np.convolve(fuera, np.ones(20), mode="full")[: len(t)] >= 20   # 20 ms seguidos
        t_det[f] = (t[np.argmax(persist)] - T_FALLA) * 1000 if persist.any() else np.nan
        sig = " ".join(" 1 " if firma[nombres.index(f), j] > 1 else " 0 " for j in range(3))
        print(f"  {f:34s}{sig}  {t_det[f]:6.0f} ms")
    falsas = sum((np.abs(base[k][int(0.5 / DT):]) > umbral[k]).sum() for k in umbral)
    print(f"  Muestras fuera de umbral sin falla (falsas alarmas crudas): {falsas}")
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.barh(list(t_det), list(t_det.values()), color=COL["alerta"])
    for i, v in enumerate(t_det.values()):
        ax.text(v, i, f" {v:.0f} ms", va="center")
    ax.set_xlabel("Tiempo desde que aparece la falla hasta la alarma (ms)")
    ax.set_title("3. Tiempo de deteccion (alarma si algun residuo sale de su banda 20 ms seguidos)")
    fig.tight_layout(); guardar(fig, SALIDA, "3_tiempos_deteccion.png")
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
