# 21.11 Digital Twin de motores eléctricos

## Por qué los motores
Los motores eléctricos consumen alrededor de la mitad de la electricidad mundial (según la Agencia Internacional de Energía) y son el activo más común en la industria. Sus fallas típicas se reparten aproximadamente así:

| Falla | Parte aproximada del total |
|---|---|
| Rodamientos | ~40 % |
| Bobinados / aislamiento | ~35 % |
| Rotor | ~10 % |
| Otras | resto |

El enemigo número 1 del aislamiento es la **temperatura**: como regla práctica, cada ~10 °C extra reduce su vida a la mitad.

## Arquitectura de este gemelo
```
FÍSICO                               GEMELO (cada 10 ms)
──────────────────────               ─────────────────────────────────────────
Motor DC + bomba                     Kalman [i, ω, T_L]  → par de carga (virtual)
MCU: PI 1 kHz, PWM 10 bits      ──►  RLS: V − Ke·ω − L·di/dt = R·i  → R en línea
Corriente (ADC 12 bits)              R(T) = R0(1+α(T−25)) → T del BOBINADO (virtual)
Encoder 1000 ppr                     Modelo Kalman actualizado con R(T) cada 1 s
Termistor en la carcasa              Potencia, eficiencia, energía
```

## Sensores virtuales: lo que el gemelo "ve" sin sensor
1. **Par de carga.** Medirlo requiere un torquímetro caro. El Kalman lo estima con ~3 mN·m de error.
2. **Temperatura del bobinado.** Es lo que se quema, pero está dentro del motor. El termistor de la carcasa llega tarde y marca menos (error ~10 °C). El gemelo usa una propiedad física:
   ```
   R(T) = R0·(1 + 0,0039·(T − 25))
   ```
   La resistencia del cobre **es** un termómetro. Estimando R en línea se obtiene T con un error de ~2,5 °C.

## Lecciones del desarrollo
- **Colinealidad.** El primer intento estimaba R y Ke juntos. Con la bomba, la corriente y la velocidad suben a la vez, los regresores son casi proporcionales y los parámetros no se pueden separar: el error de temperatura llegaba a 37 °C.

  **Solución:** usar R0, L y Kt **identificados en la puesta en marcha** (21.3) y estimar en línea **solo** lo que cambia (R por temperatura). Regla general: *no estimes en línea lo que puedes calibrar antes*.
- **No ignorar dinámicas conocidas.** El término `L·di/dt` es pequeño, pero en los transitorios sesga la estimación de R.
- **Gemelo adaptativo.** El modelo del Kalman se actualiza con R(T) cada segundo: el gemelo se mantiene fiel aunque el motor se caliente.

## Indicadores que entrega el tablero
| Indicador | Uso |
|---|---|
| Velocidad, corriente, par de carga | Operación |
| Temperatura del bobinado y margen al límite de 130 °C | Protección |
| Potencia y eficiencia | Energía y costos |
| Tendencia de R0 corregida por temperatura, Kt y fricción | Salud del motor (ver 21.14) |

## Extensión a motores de inducción y BLDC
La estructura es la misma. Cambia el modelo:
- **Motor de inducción:** ecuaciones en ejes d-q.
- **BLDC/PMSM:** control FOC, back-EMF trapezoidal o senoidal.

Los sensores virtuales de par, temperatura y flujo son la base del **control sin sensores** (*sensorless*) y del monitoreo por la firma de corriente (MCSA).

## Ejecutar
```bash
python gemelo_motor.py
```

## Ejercicios
1. Agrega una protección: si la temperatura **virtual** supera 110 °C, limitar la corriente (usa la idea de 21.1).
2. Haz que Kt baje un 10 % a mitad de la corrida (desmagnetización) y estímalo con un segundo RLS cuando haya excitación.
3. Calcula la energía que se ahorraría bajando la velocidad un 10 % (con la bomba, P ∝ ω³).
