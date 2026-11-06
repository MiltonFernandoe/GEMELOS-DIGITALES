"""
21.10 Integracion de Machine Learning con gemelos digitales
===========================================================
Dos formas de combinar ML y gemelos:

 A) EL GEMELO COMO FABRICA DE DATOS
    Las fallas reales son raras. El gemelo las simula miles de veces para
    entrenar un clasificador de diagnostico. Riesgo: la brecha simulacion-realidad.
    Solucion: "domain randomization" (variar los parametros al simular).

 B) MODELO HIBRIDO (fisica + ML)
    La fisica explica lo principal; una red pequena aprende solo lo que la
    fisica no explica (friccion no lineal, arrastre del aire...).
    Comparado con fisica sola y ML solo, sobre todo al EXTRAPOLAR.

Genera: 1_sim_a_real.png, 2_confusion.png, 3_hibrido.png
"""
import sys
import warnings
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from sklearn.ensemble import RandomForestClassifier
from sklearn.exceptions import ConvergenceWarning
from sklearn.metrics import ConfusionMatrixDisplay
from sklearn.neural_network import MLPRegressor
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.motor import MotorDC, ParamsMotor

warnings.filterwarnings("ignore", category=ConvergenceWarning)
SALIDA = carpeta_resultados(__file__)
P = ParamsMotor()
CLASES = ["normal", "rodamiento", "bobinado", "desmagnetizado", "sensor_i", "sobrecarga"]
TENSIONES = [4.0, 8.0, 12.0]           # ciclo de diagnostico: 3 escalones de tension
CARGA_ENSAYO = 0.01


def ensayo(p, clase, rng, sev):
    """Ejecuta el ciclo de diagnostico y devuelve residuos vs el modelo nominal."""
    p = p.copia()
    sesgo_i, carga = 0.0, CARGA_ENSAYO
    if clase == "rodamiento":
        p.b *= 1 + 14 * sev
    elif clase == "bobinado":
        p.R0 *= 1 + 0.4 * sev
    elif clase == "desmagnetizado":
        p.Kt *= 1 - 0.2 * sev
    elif clase == "sensor_i":
        sesgo_i = 0.3 * sev
    elif clase == "sobrecarga":
        carga += 0.03 * sev
    real, nominal = MotorDC(p), MotorDC(P)
    f = []
    for V in TENSIONES:
        i, w = real.regimen(V, carga)
        i0, w0 = nominal.regimen(V, CARGA_ENSAYO)
        i_m = i + sesgo_i + rng.normal(0, 0.01)
        w_m = w + rng.normal(0, 0.5)
        f += [i_m - i0, w_m - w0]
    return f


def generar(n, dispersion, rng):
    """dispersion: variacion relativa de los parametros entre unidades simuladas."""
    X, y = [], []
    for _ in range(n):
        c = rng.integers(len(CLASES))
        variacion = {k: getattr(P, k) * (1 + rng.normal(0, dispersion))
                     for k in ["R0", "Kt", "b", "Tc"]}
        sev = rng.uniform(0.3, 1.0)
        X.append(ensayo(P.copia(**variacion), CLASES[c], rng, sev))
        y.append(c)
    return np.array(X), np.array(y)


def parte_a(rng):
    # "flota real": las unidades varian +-6 % (tolerancias de fabricacion)
    X_real, y_real = generar(1500, 0.06, rng)
    resultados = {}
    for nombre, disp in [("gemelo ideal (sin variacion)", 0.0),
                         ("variacion pequena (1 %)", 0.01),
                         ("domain randomization (8 %)", 0.08)]:
        X_sim, y_sim = generar(4000, disp, rng)
        clf = RandomForestClassifier(n_estimators=150, random_state=0).fit(X_sim, y_sim)
        resultados[nombre] = (clf.score(X_sim[:800], y_sim[:800]) * 100,
                              clf.score(X_real, y_real) * 100, clf)
        print(f"  {nombre:32s}: precision en simulacion {resultados[nombre][0]:5.1f} % | "
              f"en la flota real {resultados[nombre][1]:5.1f} %")
    fig, ax = plt.subplots(figsize=(10, 4.5))
    x = np.arange(len(resultados))
    ax.bar(x - 0.2, [r[0] for r in resultados.values()], 0.4, color=COL["gemelo"], label="evaluado en simulacion")
    ax.bar(x + 0.2, [r[1] for r in resultados.values()], 0.4, color=COL["fisico"], label="evaluado en la flota REAL")
    for i, r in enumerate(resultados.values()):
        ax.text(i + 0.2, r[1], f"{r[1]:.0f} %", ha="center", va="bottom")
    ax.set_xticks(x); ax.set_xticklabels(list(resultados)); ax.set_ylabel("Precision (%)")
    ax.set_ylim(0, 105); ax.legend()
    ax.set_title("1. Brecha simulacion-realidad: entrenar con un gemelo 'perfecto' no generaliza; "
                 "variar parametros si")
    fig.tight_layout(); guardar(fig, SALIDA, "1_sim_a_real.png")

    clf = resultados["domain randomization (8 %)"][2]
    fig, ax = plt.subplots(figsize=(7.5, 6))
    ConfusionMatrixDisplay.from_estimator(clf, X_real, y_real, display_labels=CLASES, ax=ax,
                                          cmap="Blues", colorbar=False, xticks_rotation=30)
    ax.set_title("2. Diagnostico en la flota real (entrenado solo con datos del gemelo)")
    fig.tight_layout(); guardar(fig, SALIDA, "2_confusion.png")


def parte_b(rng):
    # El motor "real" tiene efectos que el modelo fisico simple no incluye
    def w_real(V, carga):
        m = MotorDC(P.copia(Tc=0.004))
        i, w = m.regimen(V, carga, k_cuad=6e-7)                       # + arrastre del aire
        return w * (1 - 0.04 * np.exp(-w / 40)) if w > 0 else 0.0    # + efecto no lineal a baja vel.

    def w_fisica(V, carga):
        return MotorDC(P).regimen(V, carga)[1]

    # pocos datos de entrenamiento y solo en una zona (4 a 9 V)
    V_tr = rng.uniform(4, 9, 40); C_tr = rng.uniform(0, 0.03, 40)
    y_tr = np.array([w_real(v, c) for v, c in zip(V_tr, C_tr)]) + rng.normal(0, 0.5, 40)
    X_tr = np.column_stack([V_tr, C_tr])
    fis_tr = np.array([w_fisica(v, c) for v, c in zip(V_tr, C_tr)])

    def red():
        return make_pipeline(StandardScaler(), MLPRegressor(hidden_layer_sizes=(16, 16), max_iter=5000, random_state=0,
                                                            alpha=1e-3))
    solo_ml = red().fit(X_tr, y_tr)
    residuo = red().fit(X_tr, y_tr - fis_tr)                  # el ML aprende SOLO la correccion

    V_te = np.linspace(1.5, 12, 80)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    errores = {}
    for carga, ax in zip([0.0, 0.025], axes):
        real = np.array([w_real(v, carga) for v in V_te])
        fis = np.array([w_fisica(v, carga) for v in V_te])
        X_te = np.column_stack([V_te, np.full_like(V_te, carga)])
        ml = solo_ml.predict(X_te)
        hib = fis + residuo.predict(X_te)
        ax.axvspan(4, 9, color="gray", alpha=0.12, label="zona con datos de entrenamiento")
        ax.plot(V_te, real, color=COL["fisico"], lw=3, label="motor real")
        for nombre, pred, c, est in [("solo fisica", fis, COL["modelo"], "--"),
                                     ("solo ML", ml, COL["alerta"], "-."),
                                     ("hibrido fisica + ML", hib, COL["gemelo"], "-")]:
            fuera = (V_te < 4) | (V_te > 9)
            err = np.sqrt(np.mean((pred[fuera] - real[fuera]) ** 2))
            errores.setdefault(nombre, []).append(err)
            ax.plot(V_te, pred, est, color=c, lw=1.8, label=f"{nombre} (RMS fuera de zona {err:.1f})")
        ax.set_xlabel("Tension (V)"); ax.set_ylabel("Velocidad en regimen (rad/s)")
        ax.set_title(f"Carga {carga} N*m"); ax.legend(fontsize=7)
    fig.suptitle("3. Modelo hibrido: la fisica da la forma general, el ML corrige lo no modelado, "
                 "y extrapola mucho mejor que el ML solo")
    fig.tight_layout(); guardar(fig, SALIDA, "3_hibrido.png")
    for n, e in errores.items():
        print(f"  {n:22s}: error RMS al extrapolar = {np.mean(e):5.2f} rad/s")


def main():
    titulo("21.10 Machine Learning + gemelos digitales")
    rng = np.random.default_rng(0)
    print("  A) El gemelo como fabrica de datos de fallas")
    parte_a(rng)
    print("\n  B) Modelo hibrido fisica + ML")
    parte_b(rng)
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
