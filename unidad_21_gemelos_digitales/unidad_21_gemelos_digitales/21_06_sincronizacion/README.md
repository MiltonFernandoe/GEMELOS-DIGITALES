# 21.6 Sincronización entre sistema virtual y físico

## El problema
Aunque el modelo sea bueno, el gemelo **se aleja** del sistema real por cuatro razones:

1. **Parámetros distintos:** cada unidad es diferente y además envejece.
2. **Entradas no medidas:** carga, temperatura ambiente, perturbaciones.
3. **Dinámicas no modeladas:** fricción no lineal, holguras.
4. **Estado inicial desconocido.**

Sincronizar es corregir continuamente el **estado** (y a veces los **parámetros**) del gemelo con las mediciones.

## Opciones de sincronización
| Método | Idea | Cuándo usarlo |
|---|---|---|
| **Sobrescritura** | `estado_gemelo = medición` | Estados medidos directamente y con poco ruido |
| **Observador de Luenberger** | `x̂' = f(x̂) + L·(y − ŷ)`, con ganancia fija | Sistemas lineales, ruido conocido |
| **Filtro de Kalman (KF/EKF/UKF)** | Ganancia óptima según la incertidumbre del modelo (Q) y del sensor (R) | El estándar en gemelos |
| **Filtro de partículas** | Monte Carlo | Muy no lineal o no gaussiano |
| **Estimación conjunta estado + parámetros** | Parámetros como estados extra o RLS en paralelo | Para seguir el envejecimiento (ver 21.11) |

## Filtro de Kalman en dos pasos
```
PREDECIR:  x̂ = F·x̂ + B·u           P = F·P·Fᵀ + Q        (el modelo avanza)
CORREGIR:  K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹                         (¿cuánto creer al sensor?)
           x̂ = x̂ + K·(z − H·x̂)     P = (I − K·H)·P         (se acerca a la medición)
```
- **Q grande:** "mi modelo no es confiable", así que se sigue más a los sensores.
- **R grande:** "mi sensor es ruidoso", así que se sigue más al modelo.
- **P** es la **incertidumbre del gemelo sobre sí mismo**. Es valiosa porque el gemelo *sabe* cuándo no sabe.

## Estado aumentado: sensores virtuales
Si se agrega el par de carga como un estado más, modelado como un "paseo aleatorio" (`T_L[k+1] = T_L[k] + ruido`), el filtro lo **estima sin medirlo**. Eso es un **sensor virtual** (gráfica 2).

Lo que estima incluye también la fricción no modelada, por eso el valor queda desplazado ~Tc del par real. Es correcto: es "todo el par que el modelo no explica".

## Resultados (sincronizando cada 20 ms, telemetría a 50 Hz)
| Estrategia | Error RMS de velocidad |
|---|---|
| A) Lazo abierto (misma tensión, sin mediciones) | ~30 rad/s |
| B) Kalman [i, ω] | ~4,8 rad/s |
| C) Kalman [i, ω, T_L] | **~2,0 rad/s** |

- **Con sincronización cada 1 ms, B es ligeramente mejor:** las mediciones lo corrigen todo el tiempo.
- **Entre mediciones espaciadas, C gana claramente:** su modelo explica la carga y **predice mejor** (gráfica 3).

**Lección:** cuanto menos frecuente sea la sincronización, más importa la calidad del modelo.

## Pérdida de comunicación
Durante un corte (gráfica 4), el gemelo **sigue prediciendo**, y su incertidumbre (±2σ) **crece**. Al volver los datos, se resincroniza en pocos pasos.

Un gemelo bien diseñado debe **mostrar esa incertidumbre**. No debe pintar un valor como si fuera cierto cuando lleva 600 ms sin datos.

## Ejecutar
```bash
python sincronizacion.py
```

## Ejercicios
1. Multiplica Q por 100 y luego divídela por 100. ¿Qué pasa con el ruido y con el retardo de la estimación?
2. Agrega `b` (fricción) como cuarto estado. ¿Converge a su valor real?
3. Haz que las mediciones de corriente lleguen cada 1 ms y las de velocidad cada 50 ms (multi-tasa).
4. Implementa la versión EKF con R dependiente de la temperatura.
