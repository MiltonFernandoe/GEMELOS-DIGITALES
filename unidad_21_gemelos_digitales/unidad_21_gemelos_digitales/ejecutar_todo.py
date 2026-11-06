"""
Ejecuta los 14 subtemas de la unidad en orden y la prueba del simulador en vivo.
Las graficas quedan en la carpeta resultados/ de cada subtema.

    python ejecutar_todo.py
"""
import os
import subprocess
import sys
import time
from pathlib import Path

RAIZ = Path(__file__).resolve().parent
SUBTEMAS = [
    ("21_01_fundamentos", "fundamentos.py"),
    ("21_02_arquitectura", "arquitectura.py"),
    ("21_03_modelado", "modelado.py"),
    ("21_04_mcu_virtual", "mcu_virtual.py"),
    ("21_05_sensores_actuadores", "sensores_actuadores.py"),
    ("21_06_sincronizacion", "sincronizacion.py"),
    ("21_07_tiempo_real", "tiempo_real.py"),
    ("21_08_simulacion_fallas", "simulacion_fallas.py"),
    ("21_09_prediccion", "prediccion.py"),
    ("21_10_machine_learning", "ml_gemelo.py"),
    ("21_11_motor_electrico", "gemelo_motor.py"),
    ("21_12_robot_movil", "gemelo_robot.py"),
    ("21_13_sistema_industrial", "gemelo_industrial.py"),
    ("21_14_mantenimiento_predictivo", "mantenimiento_predictivo.py"),
]


def main():
    entorno = dict(os.environ, MPLBACKEND="Agg")
    resumen = []
    for carpeta, script in SUBTEMAS:
        t0 = time.time()
        r = subprocess.run([sys.executable, script, "--sin-ventanas"], cwd=RAIZ / carpeta, env=entorno)
        resumen.append((carpeta, "OK" if r.returncode == 0 else "ERROR", time.time() - t0))
        print()
    t0 = time.time()
    r = subprocess.run([sys.executable, "simulador_en_vivo.py", "--prueba"], cwd=RAIZ, env=entorno)
    resumen.append(("simulador_en_vivo (prueba)", "OK" if r.returncode == 0 else "ERROR", time.time() - t0))

    print("=" * 60 + "\nRESUMEN\n" + "=" * 60)
    for c, e, t in resumen:
        print(f"  {c:34s}{e:>8s}{t:8.1f} s")
    return 0 if all(e == "OK" for _, e, _ in resumen) else 1


if __name__ == "__main__":
    sys.exit(main())
