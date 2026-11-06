# 21.10 Integración de Machine Learning con gemelos digitales

## Cuatro formas de combinar ML y gemelos
| Patrón | Idea | Ejemplo |
|---|---|---|
| **Gemelo → datos → ML** | El gemelo genera datos etiquetados que no existen en la realidad | Clasificador de fallas (parte A) |
| **Modelo híbrido** | La física da la estructura y el ML corrige lo no modelado | Velocidad del motor (parte B) |
| **Modelo sustituto** (*surrogate*) | Una red imita a un simulador lento para predecir en milisegundos | Optimización y tiempo real |
| **ML para sincronizar o detectar** | Detección de anomalías sobre los residuos del gemelo | Módulo 04 del curso anterior |

## A) El gemelo como fábrica de datos
Las fallas reales son escasas. El gemelo simula miles de ensayos de diagnóstico (3 tensiones, que dan 6 residuos) para 6 clases: normal, rodamiento, bobinado, desmagnetización, sensor de corriente y sobrecarga. Con esos datos se entrena un Random Forest.

### La brecha simulación-realidad (*sim-to-real gap*)
| Entrenado con | Precisión en simulación | Precisión en la flota real |
|---|---|---|
| Gemelo "perfecto" (todas las unidades iguales) | 100 % | ~67 % |
| Variación del 1 % | 100 % | ~74 % |
| **Domain randomization del 8 %** | 100 % | **~89 %** |

- **El 100 % en simulación es engañoso.** Las unidades reales difieren del nominal (±6 % de tolerancia) y el clasificador confunde esas diferencias con fallas.
- **Domain randomization:** se simula con parámetros variados, **más** que la variación real. Así el modelo aprende a ignorar lo que no importa.
- **Siempre** hay que validar con datos reales, aunque sean pocos.

## B) Modelo híbrido: física + ML
El motor real tiene arrastre de aire (∝ω²), más fricción de Coulomb y un efecto no lineal a baja velocidad: cosas que el modelo físico simple no tiene. Se entrena con solo 40 puntos entre 4 y 9 V, y se evalúa entre 1,5 y 12 V.

| Modelo | Error al extrapolar |
|---|---|
| Solo física | ~13 rad/s (sesgo sistemático) |
| Solo ML | ~27 rad/s (fuera de los datos inventa cualquier cosa) |
| **Híbrido** (`ω = física(V, carga) + ML(V, carga)`) | **~1 rad/s** |

**Por qué funciona:** el ML solo aprende la **corrección**, que es pequeña y suave. La física aporta la extrapolación.

Otras variantes de modelos híbridos:
- Ajustar los parámetros físicos con ML.
- **PINNs** (redes informadas por la física): la ecuación se agrega a la función de pérdida.
- **Neural ODEs.**

## ML dentro del MCU
El clasificador o el término de corrección pueden **desplegarse en el MCU** con las técnicas del curso de IA embebida: árbol a `if/else` en C, o una red cuantizada a int8. Así el gemelo "vive" en parte en el edge.

## Ejecutar
```bash
python ml_gemelo.py
```

## Ejercicios
1. Grafica la precisión en la flota real contra el % de randomization (0 a 20 %). ¿Hay un óptimo?
2. Agrega a la "flota real" un ruido de sensor el doble de grande que en la simulación. ¿Qué pasa?
3. Reemplaza el Random Forest por un árbol de profundidad 5 y expórtalo a C (módulo 05 del curso anterior).
4. Entrena un *surrogate* (MLP) del modelo térmico de 21.9 y compara su velocidad con la simulación.
