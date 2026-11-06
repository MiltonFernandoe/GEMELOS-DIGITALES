# 21.9 Predicción del comportamiento del sistema

## La pregunta del gemelo
> "Si seguimos así (o si hacemos X), ¿qué va a pasar?"

Predecir es **simular hacia adelante desde el estado actual sincronizado**, con un plan de entradas futuras. Requiere tres cosas:

1. **Estado inicial actual**, que da la sincronización (21.6).
2. **Modelo validado**, que da el modelado (21.3).
3. **Supuestos sobre el futuro**: plan de operación, carga, ambiente.

## Análisis "¿qué pasaría si…?" (*what-if*)
El caso: a los 6 minutos llega un pedido que exige subir un 50 % la demanda de la bomba. El gemelo evalúa tres planes:

| Plan | P(superar 130 °C) | Temperatura final (mediana) |
|---|---|---|
| A: seguir a 12 V | **~32 %** | ~126 °C |
| B: bajar a 10 V | 0 % | ~91 °C |
| C: ciclos 12 V / 6 V | ~2 % | ~66 °C |

El operador decide **con números y riesgos**, no con intuición. Esta es la esencia de un gemelo como herramienta de decisión.

## Incertidumbre: Monte Carlo
Los parámetros nunca se conocen exactos. En lugar de una sola simulación se hacen **400**, cada una con parámetros sorteados de su distribución:

| Parámetro | Incertidumbre |
|---|---|
| Rth | ±8 % |
| Cth | ±10 % |
| Carga | ±5 % |
| Temperatura ambiente | ±1,5 °C |

De esas simulaciones se obtiene:
- una **banda de confianza** (percentiles 5–95);
- la **probabilidad** de superar un límite;
- la **distribución del tiempo** hasta superarlo.

**Truco de implementación:** las 400 simulaciones se hacen **a la vez** con NumPy (vectorizado). Tardan lo mismo que una.

## Validación de la predicción
Se ejecuta el plan A en el motor "real" y la temperatura verdadera queda **dentro de la banda** el 100 % del tiempo. Una banda bien calibrada debería contener la realidad aproximadamente en el porcentaje nominal:
- si la realidad cae fuera muy seguido, la incertidumbre está **subestimada**;
- si la banda es enorme, la predicción es inútil.

## El error crece con el horizonte
Con carga y tensión que cambian sin aviso, el error medio crece con el horizonte:

| Horizonte | Error medio |
|---|---|
| 10 s | ~0,7 °C |
| 2 min | ~4 °C |
| 10 min | ~13 °C |

**Consecuencia:** el gemelo debe **re-sincronizar y re-predecir continuamente** (horizonte deslizante). Es la misma idea del control predictivo por modelo (MPC).

## Tipos de predicción en gemelos
| Horizonte | Uso |
|---|---|
| Milisegundos | Compensar latencia (21.7), control predictivo |
| Minutos | Protecciones térmicas, *what-if* operativo |
| Días o meses | Vida útil remanente y mantenimiento (21.14) |

## Ejecutar
```bash
python prediccion.py
```

## Ejercicios
1. Agrega un plan D que maximice el caudal (velocidad) sin superar un 5 % de riesgo. Búscalo automáticamente.
2. Aumenta la incertidumbre de Rth a ±20 %. ¿Cómo cambia la decisión?
3. Implementa un horizonte deslizante: cada 30 s re-predecir y ajustar la tensión (un MPC sencillo).
