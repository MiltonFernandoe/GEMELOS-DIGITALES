"""
21.14 Aplicacion en mantenimiento predictivo (caso practico final)
==================================================================
Una planta tiene 40 motores. Los rodamientos se desgastan (la friccion b crece
exponencialmente desde un dia desconocido). Cada motor tiene un gemelo que:

  1. Cada dia ejecuta un CICLO DE DIAGNOSTICO (5 tensiones sin carga) y estima b
     con  Kt*i = b*w + Tc  ->  indice de salud HI = b / b_inicial
  2. Detecta el inicio de la degradacion
  3. Ajusta el modelo de degradacion HI = c + exp(a + lam*t) y estima la
     VIDA UTIL REMANENTE (RUL)
     con intervalo de confianza (bootstrap)
  4. Programa el mantenimiento cuando RUL (pesimista, p10) < plazo de compra/planificacion

Se compara contra mantenimiento correctivo (hasta que falle) y preventivo (calendario).

Genera: 1_diagnostico.png, 2_flota.png, 3_pronostico.png, 4_estrategias.png, 5_calendario.png
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.motor import MotorDC, ParamsMotor

SALIDA = carpeta_resultados(__file__)
P = ParamsMotor()
TENSIONES = np.array([4.0, 6.0, 8.0, 10.0, 12.0])
HI_FALLA = 10.0                 # el rodamiento se agarrota cuando la friccion es 10x
HI_DETECCION = 1.6
DIAS = 500
N_MOTORES = 40
PLAZO = 21                      # dias necesarios para comprar repuesto y programar la parada
COSTOS = {"falla": 12000, "planificado": 1500, "monitoreo_anual": 150}
PARADA = {"falla": 3.0, "planificado": 0.5}


class Rodamiento:
    """Degradacion: b(d) = b0 * (1 + A*(exp(lam*(d - d0)) - 1)) a partir del dia d0."""

    def __init__(self, rng, dia_instalacion):
        self.d0 = dia_instalacion + rng.uniform(40, 260)
        self.lam = rng.uniform(0.012, 0.03)
        self.A = rng.uniform(0.4, 1.2)

    def hi(self, dia):
        if dia < self.d0:
            return 1.0
        return 1 + self.A * (np.exp(self.lam * (dia - self.d0)) - 1)

    def dia_falla(self):
        return self.d0 + np.log((HI_FALLA - 1) / self.A + 1) / self.lam


def diagnostico(hi_real, rng, detalle=False):
    """Ciclo de diagnostico en el motor + estimacion en el gemelo."""
    motor = MotorDC(P.copia(b=P.b * hi_real))
    i_m, w_m = [], []
    for V in TENSIONES:
        i, w = motor.regimen(V, 0.0)
        i_m.append(i + rng.normal(0, 0.004)); w_m.append(w + rng.normal(0, 0.3))
    i_m, w_m = np.array(i_m), np.array(w_m)
    X = np.column_stack([w_m, np.ones_like(w_m)])
    (b_est, tc_est), *_ = np.linalg.lstsq(X, P.Kt * i_m, rcond=None)
    if detalle:
        return b_est, i_m, w_m, tc_est
    return b_est


C_GRILLA = np.linspace(-0.6, 0.95, 32)


def ajustar(d, Y):
    """Modelo de degradacion HI = c + exp(a + lam*t). Para cada c candidato el ajuste es lineal
    en log(HI - c); se elige el c con menor error. Y: (n_muestras, n_puntos). Vectorizado."""
    A = np.column_stack([np.ones_like(d), d - d[-1]])
    pinv = np.linalg.pinv(A)
    mejor_sse = np.full(Y.shape[0], np.inf)
    mejor = np.zeros((Y.shape[0], 3))
    for c in C_GRILLA:
        L = np.log(np.maximum(Y - c, 1e-3))
        coef = L @ pinv.T
        sse = np.sum((np.exp(coef @ A.T) + c - Y) ** 2, axis=1)
        m = sse < mejor_sse
        mejor_sse[m] = sse[m]
        mejor[m] = np.column_stack([np.full(m.sum(), c), coef[m]])
    return mejor                                     # columnas: c, a, lam  (t medido desde el ultimo dia)


def rul(dias, hi, rng, n_boot=200, ventana=90):
    """Vida util remanente (p10, p50, p90) en dias, con bootstrap sobre el modelo completo."""
    m = hi > 1.15
    d, y = dias[m][-ventana:].astype(float), hi[m][-ventana:]
    if len(d) < 8:
        return (np.inf,) * 3
    c, a, lam = ajustar(d, y[None, :])[0]
    ajuste = c + np.exp(a + lam * (d - d[-1]))
    res = y - ajuste
    Y = ajuste[None, :] + rng.choice(res, size=(n_boot, len(res)))
    P_ = ajustar(d, Y)
    c, a, lam = P_[:, 0], P_[:, 1], P_[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        restante = np.where(lam > 1e-4, (np.log(HI_FALLA - c) - a) / lam, np.inf)
    return tuple(np.percentile(np.maximum(restante, 0), [10, 50, 90]))


def correr_estrategia(estrategia, semilla):
    rng = np.random.default_rng(semilla)
    eventos, costo, parada = [], 0.0, 0.0
    for m in range(N_MOTORES):
        rod = Rodamiento(rng, 0)
        inicio, hist_d, hist_hi = 0, [], []
        b_ref = None
        dia = 0
        while dia < DIAS:
            if dia >= rod.dia_falla():
                eventos.append((m, dia, "falla")); costo += COSTOS["falla"]; parada += PARADA["falla"]
                dia += int(PARADA["falla"])
                rod, inicio, hist_d, hist_hi, b_ref = Rodamiento(rng, dia), dia, [], [], None
                continue
            if estrategia == "preventivo" and dia - inicio >= 150:
                eventos.append((m, dia, "planificado")); costo += COSTOS["planificado"]
                parada += PARADA["planificado"]
                rod, inicio, hist_d, hist_hi, b_ref = Rodamiento(rng, dia), dia, [], [], None
            if estrategia == "predictivo":
                b = diagnostico(rod.hi(dia), rng)
                if dia - inicio < 15:
                    b_ref = b if b_ref is None else 0.9 * b_ref + 0.1 * b
                else:
                    hist_d.append(dia); hist_hi.append(b / b_ref)
                    if len(hist_hi) > 3 and min(hist_hi[-3:]) > HI_DETECCION:
                        p10, _, _ = rul(np.array(hist_d), np.array(hist_hi), rng, n_boot=100)
                        if p10 < PLAZO:
                            eventos.append((m, dia, "planificado")); costo += COSTOS["planificado"]
                            parada += PARADA["planificado"]
                            rod, inicio, hist_d, hist_hi, b_ref = Rodamiento(rng, dia), dia, [], [], None
            dia += 1
    if estrategia == "predictivo":
        costo += COSTOS["monitoreo_anual"] * N_MOTORES * DIAS / 365
    return eventos, costo, parada


def main():
    titulo("21.14 Mantenimiento predictivo con gemelos digitales")
    rng = np.random.default_rng(3)

    # --- 1. el ciclo de diagnostico
    fig, ax = plt.subplots(figsize=(8, 5))
    for hi, c, n in [(1.0, COL["ok"], "rodamiento nuevo"), (4.0, COL["alerta"], "desgastado (b x4)"),
                     (9.0, COL["falla"], "casi falla (b x9)")]:
        b_est, i_m, w_m, tc = diagnostico(hi, rng, detalle=True)
        ax.plot(w_m, P.Kt * i_m * 1000, "o", color=c, ms=8)
        ww = np.linspace(0, 260, 10)
        ax.plot(ww, (b_est * ww + tc) * 1000, color=c,
                label=f"{n}: b estimado = {b_est * 1e5:.1f}e-5 (real {P.b * hi * 1e5:.1f}e-5)")
    ax.set_xlabel("Velocidad en vacio (rad/s)"); ax.set_ylabel("Par motor = Kt * i (mN*m)")
    ax.set_title("1. Diagnostico diario: 5 tensiones en vacio. La pendiente es la friccion b")
    ax.legend(fontsize=8); fig.tight_layout(); guardar(fig, SALIDA, "1_diagnostico.png")

    # --- 2. flota (sin intervenir) y 3. pronostico de un motor
    fig, ax = plt.subplots(figsize=(11, 5))
    rods = [Rodamiento(rng, 0) for _ in range(12)]
    dias = np.arange(DIAS)
    for k, rod in enumerate(rods):
        b_ref = np.mean([diagnostico(1.0, rng) for _ in range(10)])
        hi = np.array([diagnostico(rod.hi(d), rng) / b_ref for d in dias])
        corte = min(DIAS, int(rod.dia_falla()) + 1)
        ax.semilogy(dias[:corte], hi[:corte], lw=1)
        if corte < DIAS:
            ax.plot(corte - 1, hi[corte - 1], "x", color=COL["falla"], ms=8, mew=2)
    ax.axhline(HI_FALLA, color=COL["falla"], ls="--", label="falla (HI = 10)")
    ax.axhline(HI_DETECCION, color=COL["alerta"], ls=":", label="deteccion de degradacion")
    ax.set_xlabel("Dia"); ax.set_ylabel("Indice de salud HI = b / b_inicial (log)")
    ax.set_title("2. 12 motores estimados por sus gemelos: cada rodamiento empieza a fallar un dia distinto y a otra velocidad")
    ax.legend(); fig.tight_layout(); guardar(fig, SALIDA, "2_flota.png")

    rod = Rodamiento(np.random.default_rng(11), 0)
    b_ref = np.mean([diagnostico(1.0, rng) for _ in range(10)])
    d_falla = rod.dia_falla()
    dias_m = np.arange(int(d_falla))
    hi_m = np.array([diagnostico(rod.hi(d), rng) / b_ref for d in dias_m])
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5.2))
    ax1.semilogy(dias_m, hi_m, ".", color=COL["fisico"], ms=3, label="HI estimado cada dia")
    ax1.axhline(HI_FALLA, color=COL["falla"], ls="--")
    deteccion = dias_m[np.argmax(np.convolve(hi_m > HI_DETECCION, np.ones(3), "full")[:len(hi_m)] >= 3)]
    for frac, c in [(0.25, COL["pred"]), (0.6, COL["gemelo"]), (0.85, COL["ok"])]:
        d_an = int(deteccion + frac * (d_falla - deteccion))
        p10, p50, p90 = rul(dias_m[:d_an + 1], hi_m[:d_an + 1], rng)
        m = hi_m[:d_an + 1] > 1.15
        d_fit, y_fit = dias_m[:d_an + 1][m][-90:].astype(float), hi_m[:d_an + 1][m][-90:]
        cc, aa, ll = ajustar(d_fit, y_fit[None, :])[0]
        dd = np.linspace(d_fit[0], d_an + (p90 if np.isfinite(p90) else 100), 60)
        ax1.semilogy(dd, cc + np.exp(aa + ll * (dd - d_fit[-1])), "--", color=c)
        if np.isfinite(p90):
            ax1.axvspan(d_an + p10, d_an + p90, color=c, alpha=0.15)
        ax1.axvline(d_an, color=c, lw=0.8)
        ax1.text(d_an, 1.05, f" dia {d_an}\n RUL {p50:.0f} d\n [{p10:.0f}-{p90:.0f}]", color=c, fontsize=8)
    ax1.axvline(d_falla, color=COL["falla"], lw=2, label=f"falla real (dia {d_falla:.0f})")
    ax1.set_xlim(deteccion - 60, d_falla + 30); ax1.set_ylim(0.8, 30)
    ax1.set_xlabel("Dia"); ax1.set_ylabel("HI (log)"); ax1.legend(fontsize=8)
    ax1.set_title("Pronostico en tres momentos (franjas = intervalo p10-p90 de la falla)")
    # RUL vs tiempo
    d_eval = np.arange(deteccion + 5, int(d_falla))
    pred = np.array([rul(dias_m[:d + 1], hi_m[:d + 1], rng, n_boot=150) for d in d_eval])
    real = d_falla - d_eval
    ax2.plot(d_eval, real, color=COL["fisico"], lw=2, label="RUL real")
    ax2.fill_between(d_eval, real * 0.8, real * 1.2, color="gray", alpha=0.15, label="+-20 % (cono alfa-lambda)")
    ax2.plot(d_eval, pred[:, 1], color=COL["gemelo"], label="RUL estimada (p50)")
    ax2.fill_between(d_eval, pred[:, 0], pred[:, 2], color=COL["gemelo"], alpha=0.2, label="p10-p90")
    ax2.axhline(PLAZO, color=COL["alerta"], ls=":", label=f"plazo de planificacion ({PLAZO} d)")
    ax2.set_ylim(0, max(real) * 1.6); ax2.set_xlabel("Dia"); ax2.set_ylabel("Dias hasta la falla")
    ax2.legend(fontsize=8); ax2.set_title("La estimacion converge a la realidad a medida que llegan datos")
    fig.suptitle("3. Pronostico de vida util remanente (RUL) de un motor")
    fig.tight_layout(); guardar(fig, SALIDA, "3_pronostico.png")
    ultimo = pred[-PLAZO]
    print(f"  Motor de ejemplo: degradacion detectada el dia {deteccion}, falla real el dia {d_falla:.0f}")
    print(f"  {PLAZO} dias antes de la falla el gemelo estimaba RUL = {ultimo[1]:.0f} d "
          f"[{ultimo[0]:.0f} - {ultimo[2]:.0f}]")

    # --- 4. estrategias en la flota
    print(f"\n  Simulando {N_MOTORES} motores x {DIAS} dias con 3 estrategias...")
    resultados = {}
    for e in ["correctivo", "preventivo", "predictivo"]:
        ev, costo, parada = correr_estrategia(e, semilla=7)
        fallas = sum(1 for x in ev if x[2] == "falla")
        plan = sum(1 for x in ev if x[2] == "planificado")
        resultados[e] = dict(eventos=ev, costo=costo, parada=parada, fallas=fallas, plan=plan)
        print(f"  {e:11s}: fallas {fallas:3d} | intervenciones planificadas {plan:3d} | "
              f"dias de parada {parada:6.1f} | costo {costo:9,.0f}")
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    nombres = list(resultados)
    cols = [COL["falla"], COL["alerta"], COL["ok"]]
    for ax, (clave, etiqueta) in zip(axes, [("fallas", "Fallas imprevistas"), ("parada", "Dias de parada"),
                                             ("costo", "Costo total ($)")]):
        vals = [resultados[n][clave] for n in nombres]
        ax.bar(nombres, vals, color=cols)
        for i, v in enumerate(vals):
            ax.text(i, v, f"{v:,.0f}", ha="center", va="bottom")
        ax.set_title(etiqueta)
    fig.suptitle(f"4. {N_MOTORES} motores durante {DIAS} dias: el mantenimiento predictivo evita casi todas "
                 "las fallas sin cambiar rodamientos sanos")
    fig.tight_layout(); guardar(fig, SALIDA, "4_estrategias.png")

    # --- 5. calendario de intervenciones (predictivo vs correctivo)
    fig, axes = plt.subplots(1, 2, figsize=(15, 6), sharey=True)
    for ax, e in zip(axes, ["correctivo", "predictivo"]):
        for m, d, tipo in resultados[e]["eventos"]:
            if m < 20:
                ax.plot(d, m, "X" if tipo == "falla" else "o", color=COL["falla"] if tipo == "falla" else COL["ok"],
                        ms=9 if tipo == "falla" else 7)
        ax.set_title(f"{e}: {resultados[e]['fallas']} fallas, {resultados[e]['plan']} planificadas (todos los motores)")
        ax.set_xlabel("Dia"); ax.set_xlim(0, DIAS)
    axes[0].set_ylabel("Motor #")
    axes[1].plot([], [], "o", color=COL["ok"], label="cambio planificado")
    axes[1].plot([], [], "X", color=COL["falla"], label="falla imprevista")
    axes[1].legend(loc="upper right")
    fig.suptitle("5. Calendario de los primeros 20 motores")
    fig.tight_layout(); guardar(fig, SALIDA, "5_calendario.png")
    ahorro = resultados["correctivo"]["costo"] - resultados["predictivo"]["costo"]
    print(f"\n  Ahorro del predictivo frente al correctivo: {ahorro:,.0f} "
          f"({ahorro / resultados['correctivo']['costo'] * 100:.0f} %)")
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
