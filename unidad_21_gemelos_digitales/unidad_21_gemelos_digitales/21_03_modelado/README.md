# 21.3 Modelado de un dispositivo físico en Python

## Tres formas de modelar
| Tipo | Cómo | Ventaja | Desventaja |
|---|---|---|---|
| **Caja blanca** (primeros principios) | Leyes físicas | Interpretable y extrapola bien | Requiere conocer la física y los parámetros |
| **Caja negra** (basado en datos) | ML, ARX, redes | No necesita física | Necesita muchos datos y no extrapola |
| **Caja gris** (híbrido) | Física + parámetros ajustados con datos | Lo mejor de ambos | Más trabajo de diseño |

En gemelos digitales se usa sobre todo la **caja gris**: la estructura viene de la física y los parámetros se ajustan a *esta* unidad con sus datos.

## A) Ecuaciones del motor DC (ver `gemelo/motor.py`)
```
L·di/dt  = V − R(T)·i − Ke·ω                      eléctrica  (Ley de Kirchhoff)
J·dω/dt  = Kt·i − b·ω − Tc·tanh(ω/ω0) − T_carga   mecánica   (2ª ley de Newton)
Cth·dT/dt = R(T)·i² − (T − T_amb)/Rth             térmica    (balance de energía)
R(T)      = R0·(1 + α·(T − 25))                   el cobre sube 0,39 %/°C
```
En Python, el modelo es una clase con su **estado** (`i, ω, θ, T`) y un método `paso(V, dt)`. Ese mismo patrón sirve para cualquier dispositivo.

**Escalas de tiempo muy distintas** (sistema rígido):

| Dinámica | Constante de tiempo |
|---|---|
| Eléctrica | L/R ≈ 2,5 ms |
| Mecánica | J·R/Kt² ≈ 80 ms |
| Térmica | Rth·Cth ≈ 60 s |

Para horizontes largos se usa `paso_lento`, que resuelve lo rápido en régimen permanente y solo integra lo lento.

## B) Integración numérica
- **Euler:** `x += dt·f(x)`. Es simple, pero es de orden 1 y se vuelve inestable si `dt` es comparable a la constante de tiempo más rápida.
- **Runge-Kutta 4 (RK4):** evalúa `f` cuatro veces por paso y es de orden 4. Al reducir `dt` a la mitad, el error baja 16 veces.
- **Regla práctica:** `dt` ≤ 1/10 de la constante de tiempo más rápida, o usar RK4.

La gráfica 2 muestra las pendientes 1 y 4 en escala log-log.

## C) Identificación de parámetros
Los datos de catálogo tienen tolerancias de ±10–20 %, así que se hace un **ensayo** con una entrada que excite todas las dinámicas (señal multinivel o PRBS) y se ajusta por mínimos cuadrados.

**Etapa 1, eléctrica:** es exacta en tiempo discreto con retención de orden cero (ZOH):
```
i[k+1] = a·i[k] + cV·V[k] + cω·ω[k]     →   R = (1−a)/cV,   L = −R·dt/ln(a),   Kt = −cω/cV
```

**Etapa 2, mecánica en forma integral:**
```
J·(ω[k+n] − ω[k]) = Kt·∫i − b·∫ω − Tc·∫sign(ω)
```
Integrar sobre una ventana promedia el ruido del sensor. Derivar la velocidad medida, en cambio, lo amplifica. El primer intento de este ejemplo, que usaba la forma discreta directa, estimó `b` con un 990 % de error por el ruido en los regresores (el problema de los "errores en las variables").

**Resultado:**
- R, L, Kt y J quedan con menos del 1 % de error.
- `b` y `Tc` quedan con 10–20 % de error: los dos son fricciones y se parecen en los datos (colinealidad). Para separarlos hace falta excitar el motor en un rango amplio de velocidades.
- **Validación con una entrada nunca vista:** el modelo de catálogo ajusta un 82,6 % y el identificado un **99,7 %**.

## Validación: no lo olvides
- Se valida **siempre con datos que no se usaron para ajustar.**
- **Métrica NRMSE (ajuste %):** `100·(1 − ‖y − ŷ‖ / ‖y − ȳ‖)`.
- Hay que probar condiciones distintas (carga, temperatura) a las del ensayo.

## Ejecutar
```bash
python modelado.py
```

## Ejercicios
1. Agrega ruido de 1 rad/s al sensor de velocidad. ¿Qué parámetros se degradan más?
2. Haz el ensayo solo a 10 y 12 V. ¿Qué pasa con `b` y `Tc`?
3. Identifica también el modelo térmico (Rth, Cth) con un ensayo de calentamiento.
4. Exporta el modelo como FMU con la librería `pythonfmu` para usarlo en otras herramientas.
