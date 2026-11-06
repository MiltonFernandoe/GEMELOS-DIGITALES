"""Utilidades comunes para las graficas de la unidad."""
import sys
from pathlib import Path

import matplotlib.pyplot as plt

plt.rcParams.update({
    "figure.dpi": 100, "axes.grid": True, "grid.alpha": 0.3,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.titlesize": 11, "font.size": 9,
})

COL = {"fisico": "#222222", "gemelo": "#1f77b4", "modelo": "#d62728", "medido": "#9e9e9e",
       "ok": "#2ca02c", "alerta": "#ff7f0e", "falla": "#d62728", "pred": "#9467bd"}


def carpeta_resultados(archivo):
    salida = Path(archivo).resolve().parent / "resultados"
    salida.mkdir(exist_ok=True)
    return salida


def guardar(fig, salida, nombre):
    fig.savefig(salida / nombre, dpi=110, bbox_inches="tight")


def mostrar():
    if "--sin-ventanas" not in sys.argv:
        plt.show()


def titulo(texto):
    print("=" * 70 + f"\n{texto}\n" + "=" * 70)
