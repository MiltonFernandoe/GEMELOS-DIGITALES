# 21.5 Simulación de sensores y actuadores

## Por qué importa
El gemelo recibe **mediciones**, no la realidad. Si su modelo de sensor es ideal:
- confunde un **defecto del sensor** con un **cambio del proceso** (por ejemplo, una deriva parece un calentamiento);
- es demasiado optimista sobre la calidad del control;
- no sabe cuánto confiar en cada dato, que es justo lo que el filtro de Kalman necesita (matriz R).

## Modelo general de un sensor
```
medición = cuantizar( saturar( g·x(t − τ) + offset + deriva·t + ruido ) )
```

| Defecto | Causa típica | Cómo se ve |
|---|---|---|
| Ruido | Electrónica, EMI | Dispersión aleatoria |
| Offset y ganancia | Calibración, tolerancias | Error sistemático |
| Cuantización | ADC de pocos bits | Escalones |
| Retardo | Filtro interno, masa térmica, bus lento | La medición va atrasada |
| Saturación | Rango insuficiente | La señal se aplana |
| Deriva | Envejecimiento, temperatura | Error que crece con el tiempo |

`gemelo/sensores.py` implementa todos estos defectos, y además **fallas inyectables** (sesgo, atascado, pérdida de datos…) que se usan en 21.8.

## Encoder: medir velocidad
Un encoder de cuadratura con 100 ppr da 400 cuentas por vuelta. Hay dos métodos (gráfica 2):

- **Conteo** (cuentas en Ts): `ω = Δcuentas·2π/(cpr·Ts)`.
  - Resolución fija: 2π/(cpr·Ts), es decir 15,7 rad/s con Ts = 1 ms.
  - **Malo a baja velocidad.**
- **Periodo** (tiempo entre flancos con un timer de 1 MHz): `ω = 2π/(cpr·T_flanco)`.
  - **Malo a alta velocidad**, porque hay pocos ticks entre flancos.
- **En la práctica:** se combinan los dos métodos (*M/T*) o se usa un observador, como el filtro de Kalman de 21.6.

## Actuadores
El puente H no entrega exactamente la tensión pedida:

- **Zona muerta:** por debajo de cierto ciclo de trabajo, el motor no recibe nada (tiempos muertos del driver, umbral de los MOSFET).
- **Cuantización del PWM:** con 8 bits hay 256 niveles.
- **Límite de cambio** (*slew rate*) y **saturación**.

**El ejemplo:** el motor sigue una referencia senoidal que avanza y retrocede.
- Al cruzar por cero, la zona muerta lo deja quieto un instante y el error de seguimiento sube de 8,7 a 14 rad/s RMS.
- La **compensación en firmware** (`u' = sign(u)·(|u|·(1−zm) + zm)`) lo devuelve a 8,7.
- El gemelo permite **ajustar esa compensación antes** de tocar el hardware.

## Ejecutar
```bash
python sensores_actuadores.py
```

## Ejercicios
1. Implementa el método M/T para el encoder y grafica su error en todo el rango.
2. Modela un termopar con constante de tiempo de 2 s (filtro de primer orden) y observa el retardo en una rampa.
3. Agrega histéresis mecánica (*backlash*) a un actuador de posición.
4. ¿Qué pasa si sobrecompensas la zona muerta (usas 20 % cuando la real es 15 %)?
