"""
21.12 Digital Twin de un robot movil
====================================
Robot de traccion diferencial que debe recorrer un cuadrado de 2 m dos veces.
El robot real tiene ruedas de radios distintos, una base algo mayor y
deslizamiento: su odometria (lo que calcula el MCU con los encoders) se desvia.

  Escenario A: el MCU navega solo con odometria.
  Escenario B: el GEMELO fusiona odometria + balizas de posicion (1 Hz, +-5 cm)
               con un EKF, APRENDE el error sistematico de las ruedas y le
               devuelve al robot la pose corregida (bidireccional).
               Ademas predice la trayectoria de los proximos 3 s con incertidumbre.

Genera: 1_trayectorias.png, 2_errores.png, 3_prediccion.png, 4_animacion.gif
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Ellipse

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gemelo.estimacion import EKF
from gemelo.graficos import COL, carpeta_resultados, guardar, mostrar, titulo
from gemelo.robot import ParamsRobot, RobotDiferencial, ir_a_punto, odometria

SALIDA = carpeta_resultados(__file__)
DT = 0.02
P_NOM = ParamsRobot()
P_REAL = P_NOM.copia(r_izq=0.0333, r_der=0.0327, base=0.155, desliz=0.02)
CPR = 1200                                            # cuentas del encoder por vuelta de rueda
PUNTOS = [(2, 0), (2, 2), (0, 2), (0, 0)] * 2
SIGMA_BALIZA = 0.05


# Estado del EKF: [x, y, th, kd]
#   kd = curvatura parasita (rad por metro) causada por la diferencia de radios de rueda.
#   El gemelo la APRENDE en linea: es una autocalibracion de su propio modelo.
def f_mov(x, u, dt):
    ds, dth = u
    dth_c = dth + x[3] * ds
    th = x[2] + dth_c / 2
    nuevo = x + np.array([ds * np.cos(th), ds * np.sin(th), dth_c, 0.0])
    nuevo[2] = (nuevo[2] + np.pi) % (2 * np.pi) - np.pi
    return nuevo


def jac_mov(x, u, dt):
    ds, dth = u
    th = x[2] + (dth + x[3] * ds) / 2
    return np.array([[1, 0, -ds * np.sin(th), -ds * np.sin(th) * ds / 2],
                     [0, 1, ds * np.cos(th), ds * np.cos(th) * ds / 2],
                     [0, 0, 1, ds],
                     [0, 0, 0, 1]])


def simular(con_gemelo, semilla=0, guardar_hist=False):
    rng = np.random.default_rng(semilla)
    robot = RobotDiferencial(P_REAL, semilla=semilla)
    pose_odo = np.zeros(3)
    ekf = EKF(f_mov, jac_mov, np.zeros((4, 4)), np.zeros(4), np.diag([1e-4, 1e-4, 1e-4, 0.3 ** 2]))
    cuenta_prev = np.zeros(2)
    objetivo, t, llegadas = 0, 0.0, []
    h = {k: [] for k in ["t", "real", "odo", "ekf", "sig", "baliza", "P"]}
    while objetivo < len(PUNTOS) and t < 150:
        estimada = ekf.x[:3] if con_gemelo else pose_odo
        wi, wd, dist = ir_a_punto(estimada, PUNTOS[objetivo], p=P_NOM)
        if dist < 0.06:
            llegadas.append(np.hypot(robot.x - PUNTOS[objetivo][0], robot.y - PUNTOS[objetivo][1]))
            objetivo += 1
            continue
        robot.paso(wi, wd, DT)
        # encoders cuantizados (MCU)
        cuentas = np.floor(np.array([robot.ang_i, robot.ang_d]) / (2 * np.pi) * CPR)
        d_ang = (cuentas - cuenta_prev) * 2 * np.pi / CPR
        cuenta_prev = cuentas
        pose_odo, ds, dth = odometria(pose_odo, d_ang[0], d_ang[1], P_NOM)
        # gemelo: EKF
        ekf.Q = np.diag([(0.03 * abs(ds)) ** 2 + 1e-8, (0.03 * abs(ds)) ** 2 + 1e-8,
                         (0.06 * abs(dth) + 0.03 * abs(ds)) ** 2 + 1e-8, 1e-9])
        ekf.predecir((ds, dth), DT)
        baliza = None
        if int(round(t / DT)) % int(1.0 / DT) == 0 and t > 0:
            baliza = np.array([robot.x, robot.y]) + rng.normal(0, SIGMA_BALIZA, 2)
            H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
            ekf.corregir(baliza, lambda x: x[:2], H, np.eye(2) * SIGMA_BALIZA ** 2)
        if guardar_hist or True:
            h["t"].append(t); h["real"].append(robot.pose.copy()); h["odo"].append(pose_odo.copy())
            h["ekf"].append(ekf.x.copy()); h["sig"].append(np.sqrt(np.diag(ekf.P)))
            h["baliza"].append(baliza); h["P"].append(ekf.P[:2, :2].copy())
        t += DT
    h["kd"] = [e[3] for e in h["ekf"]]
    for k in ["real", "odo", "ekf", "sig"]:
        h[k] = np.array(h[k])
    h["t"] = np.array(h["t"]); h["llegadas"] = llegadas
    return h


def predecir(pose, P, objetivo_idx, horizonte=3.0, n=25, rng=None):
    """El gemelo simula los proximos segundos con su modelo (Monte Carlo sobre pose y parametros)."""
    rng = rng or np.random.default_rng(0)
    trayectorias = []
    for _ in range(n):
        p = P_NOM.copia(r_izq=P_NOM.r_izq * (1 + rng.normal(0, 0.02)),
                        r_der=P_NOM.r_der * (1 + rng.normal(0, 0.02)),
                        base=P_NOM.base * (1 + rng.normal(0, 0.03)))
        x0 = rng.multivariate_normal(pose, P + np.eye(3) * 1e-9)
        r = RobotDiferencial(p, *x0)
        obj, tray = objetivo_idx, []
        for _ in range(int(horizonte / DT)):
            wi, wd, dist = ir_a_punto(r.pose, PUNTOS[obj], p=P_NOM)
            if dist < 0.06 and obj < len(PUNTOS) - 1:
                obj += 1
            r.paso(wi, wd, DT)
            tray.append(r.pose[:2].copy())
        trayectorias.append(np.array(tray))
    return trayectorias


def elipse(ax, centro, P, color):
    val, vec = np.linalg.eigh(P)
    ang = np.degrees(np.arctan2(vec[1, 1], vec[0, 1]))
    e = Ellipse(centro, 2 * 2 * np.sqrt(val[1]), 2 * 2 * np.sqrt(val[0]), angle=ang,
                fc=color, alpha=0.2, ec=color)
    ax.add_patch(e)
    return e


def main():
    titulo("21.12 Digital Twin de un robot movil")
    A = simular(False)
    B = simular(True)
    for n, h in [("A solo odometria", A), ("B con gemelo (EKF)", B)]:
        print(f"  {n:22s}: error al llegar a cada punto = "
              + ", ".join(f"{e * 100:.0f}" for e in h["llegadas"]) + " cm")
    errA = np.hypot(*(A["real"][:, :2] - A["odo"][:, :2]).T)
    errB = np.hypot(*(B["real"][:, :2] - B["ekf"][:, :2]).T)
    print(f"  Error de la pose que cree el robot: A max {errA.max() * 100:.0f} cm | B max {errB.max() * 100:.0f} cm")
    print(f"  Curvatura parasita aprendida por el gemelo: {B['kd'][-1]:.3f} rad/m "
          f"(radios reales {P_REAL.r_izq * 1000:.1f} / {P_REAL.r_der * 1000:.1f} mm)")

    # --- 1. trayectorias
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.5))
    cuadro = np.array(PUNTOS[:4] + [PUNTOS[0]])
    cuadro = np.vstack([[0, 0], cuadro])
    for ax, h, nombre, est, col in [(axes[0], A, "A: el robot navega solo con odometria", "odo", COL["modelo"]),
                                     (axes[1], B, "B: el gemelo corrige la pose (EKF + balizas)", "ekf", COL["gemelo"])]:
        ax.plot(cuadro[:, 0], cuadro[:, 1], "k:", lw=1, label="camino deseado")
        ax.plot(h["real"][:, 0], h["real"][:, 1], color=COL["fisico"], lw=2, label="trayectoria REAL")
        ax.plot(h[est][:, 0], h[est][:, 1], "--", color=col, lw=1.5, label="donde el robot CREE que esta")
        if est == "ekf":
            bal = np.array([b for b in h["baliza"] if b is not None])
            ax.plot(bal[:, 0], bal[:, 1], ".", color=COL["alerta"], ms=4, label="balizas (1 Hz)")
        ax.plot(*zip(*PUNTOS[:4]), "s", color="k", ms=8)
        ax.set_aspect("equal"); ax.set_title(nombre); ax.legend(fontsize=8, loc="center")
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    fig.suptitle("1. Radios de rueda con 3 % de diferencia y deslizamiento: la odometria sola se pierde")
    fig.tight_layout(); guardar(fig, SALIDA, "1_trayectorias.png")

    # --- 2. errores
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(A["t"], errA * 100, color=COL["modelo"], label="A: odometria")
    ax.plot(B["t"], errB * 100, color=COL["gemelo"], label="B: gemelo (EKF)")
    ax.plot(B["t"], 2 * np.hypot(B["sig"][:, 0], B["sig"][:, 1]) * 100, ":", color=COL["gemelo"],
            label="B: incertidumbre que el gemelo reporta (2 sigma)")
    ax.set_xlabel("Tiempo (s)"); ax.set_ylabel("Error de posicion (cm)"); ax.legend()
    ax.set_title("2. El error de odometria crece sin limite; el del gemelo queda acotado y es 'consciente' de si mismo")
    fig.tight_layout(); guardar(fig, SALIDA, "2_errores.png")

    # --- 3. prediccion
    fig, ax = plt.subplots(figsize=(8, 7.5))
    ax.plot(cuadro[:, 0], cuadro[:, 1], "k:", lw=1)
    ax.plot(B["real"][:, 0], B["real"][:, 1], color=COL["fisico"], lw=1, alpha=0.4)
    rng = np.random.default_rng(1)
    obj_idx = 0
    for k in range(0, len(B["t"]), int(6 / DT)):
        pose = B["ekf"][k][:3]
        while obj_idx < len(PUNTOS) - 1 and np.hypot(*(pose[:2] - PUNTOS[obj_idx])) < 0.3:
            obj_idx += 1
        objetivo = min(range(obj_idx, len(PUNTOS)), key=lambda j: 0 if j == obj_idx else 1)
        P3 = np.diag(B["sig"][k][:3] ** 2)
        for tr in predecir(pose, P3, objetivo, rng=rng):
            ax.plot(tr[:, 0], tr[:, 1], color=COL["pred"], lw=0.5, alpha=0.4)
        ax.plot(*pose[:2], "o", color=COL["gemelo"], ms=6)
        fin = int(min(k + 3 / DT, len(B["t"]) - 1))
        ax.plot(*B["real"][fin, :2], "x", color=COL["fisico"], ms=9, mew=2)
        if B["t"][k] > 40:
            break
    ax.plot([], [], color=COL["pred"], label="trayectorias predichas a 3 s (Monte Carlo)")
    ax.plot([], [], "o", color=COL["gemelo"], label="pose del gemelo al predecir")
    ax.plot([], [], "x", color=COL["fisico"], mew=2, label="donde estaba realmente 3 s despues")
    ax.set_aspect("equal"); ax.legend(loc="center", fontsize=8)
    ax.set_title("3. Prediccion: el gemelo simula el futuro cercano (util para evitar colisiones)")
    fig.tight_layout(); guardar(fig, SALIDA, "3_prediccion.png")

    # --- 4. animacion GIF
    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    ax.set_xlim(-0.6, 2.6); ax.set_ylim(-0.6, 2.6); ax.set_aspect("equal")
    ax.plot(cuadro[:, 0], cuadro[:, 1], "k:", lw=1)
    l_real, = ax.plot([], [], color=COL["fisico"], lw=2, label="real")
    l_ekf, = ax.plot([], [], "--", color=COL["gemelo"], lw=1.5, label="gemelo")
    p_real, = ax.plot([], [], "o", color=COL["fisico"], ms=9)
    p_ekf, = ax.plot([], [], "o", color=COL["gemelo"], ms=6)
    p_bal, = ax.plot([], [], "*", color=COL["alerta"], ms=12, label="baliza")
    texto = ax.text(-0.5, 2.45, "", fontsize=9)
    ax.legend(loc="center", fontsize=8)
    estado = {"elipse": None}
    paso = int(0.6 / DT)
    cuadros = range(0, len(B["t"]), paso)

    def actualizar(k):
        l_real.set_data(B["real"][:k + 1, 0], B["real"][:k + 1, 1])
        l_ekf.set_data(B["ekf"][:k + 1, 0], B["ekf"][:k + 1, 1])
        p_real.set_data([B["real"][k, 0]], [B["real"][k, 1]])
        p_ekf.set_data([B["ekf"][k, 0]], [B["ekf"][k, 1]])
        ultimas = [b for b in B["baliza"][max(0, k - paso):k + 1] if b is not None]
        p_bal.set_data(*([[ultimas[-1][0]], [ultimas[-1][1]]] if ultimas else [[], []]))
        if estado["elipse"]:
            estado["elipse"].remove()
        estado["elipse"] = elipse(ax, B["ekf"][k, :2], B["P"][k] * 25, COL["gemelo"])
        texto.set_text(f"t = {B['t'][k]:5.1f} s   error = {errB[k] * 100:4.1f} cm\n(elipse: incertidumbre x5)")
        return l_real, l_ekf, p_real, p_ekf, p_bal, texto

    anim = FuncAnimation(fig, actualizar, frames=cuadros, blit=False)
    anim.save(SALIDA / "4_animacion.gif", writer=PillowWriter(fps=10), dpi=70)
    plt.close(fig)
    print(f"  Animacion: {SALIDA / '4_animacion.gif'}")
    print(f"  Graficas en {SALIDA}")
    mostrar()


if __name__ == "__main__":
    main()
