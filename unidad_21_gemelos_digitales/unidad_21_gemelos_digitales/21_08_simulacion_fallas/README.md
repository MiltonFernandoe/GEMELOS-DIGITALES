# 21.8 Simulación de fallas

## ¿Por qué simular fallas?
- **Las fallas reales son raras, caras y peligrosas de provocar.** Nadie quema un motor para ver cómo se ve.
- El gemelo permite **inyectarlas virtualmente** para:
  1. diseñar y probar algoritmos de **detección y diagnóstico** (FDI: *Fault Detection and Isolation*);
  2. generar **datos de entrenamiento** para ML (ver 21.10);
  3. verificar que el firmware **reacciona bien**: protecciones, modo degradado (*fail-safe*);
  4. hacer **análisis de riesgos** (FMEA) con números.

## Taxonomía de fallas
| Tipo | Ejemplos | En `gemelo/` |
|---|---|---|
| **Sensor** | Sesgo, ganancia, atascado, ruido, pérdida de datos, deriva | `Sensor.falla` |
| **Actuador** | Pérdida de ganancia, circuito abierto, atascado | `PuenteH.falla` |
| **Componente o proceso** | Rodamiento (más fricción), bobinado (cambia R), desmagnetización (baja Kt) | parámetros del `MotorDC` |
| **Entorno o uso** | Sobrecarga, temperatura ambiente | entradas |

Por su evolución en el tiempo, una falla puede ser **abrupta** (escalón), **incipiente** (crece lentamente, ver 21.14) o **intermitente**.

## Detección por residuos
Un **residuo** es la diferencia entre lo que se mide y lo que el gemelo espera. Vale ~0 si todo está bien.

| Residuo | Fórmula | Qué lo afecta |
|---|---|---|
| `r_i` | `i_medida − i_gemelo` | Corriente |
| `r_ω` | `ω_medida − ω_gemelo` | Velocidad |
| `r_e` (ARR eléctrica) | `V_mando − R·i_med − Ke·ω_med` | Solo depende de la parte eléctrica, sensores y actuador |

Una **relación de redundancia analítica** (ARR) es una ecuación física que debe cumplirse con las mediciones. Si `r_e` ≠ 0, falla algo **eléctrico**, un **sensor** o el **actuador**, nunca lo mecánico. Así se *aísla* la falla.

## Matriz de firmas
| Falla | r_i | r_ω | r_e |
|---|---|---|---|
| Sensor de corriente con sesgo | 1 | 0 | 1 |
| Sensor de corriente atascado | 1 | 0 | 0 |
| Encoder pierde pulsos | 0 | 1 | 1 |
| Puente H con 80 % de ganancia | 1 | 1 | 1 |
| Rodamiento (fricción ×15) | 1 | 1 | **0** |
| Bobinado (R +30 %) | 0 | 1 | 1 |
| Sobrecarga | 1 | 1 | **0** |

- **Las firmas distintas** permiten saber **qué** falló, no solo **que** algo falló.
- **Las filas iguales** (rodamiento y sobrecarga, encoder y bobinado) **no se pueden aislar** con estos residuos. Hace falta otro: por ejemplo, la fricción depende de la velocidad y la sobrecarga no. Diseñar residuos para separar firmas es parte del trabajo.

## Lecciones del ejemplo
1. **Excitación:** un sensor **atascado** en régimen permanente es invisible: el valor congelado coincide con el real. Por eso a los 3 s se cambia la consigna. En la práctica se usan pruebas periódicas o se aprovechan los cambios de operación.
2. **Los umbrales se calibran con operación normal *representativa***. En una primera versión se calcularon solo con régimen estable, y el cambio de consigna generó 255 falsas alarmas. Con datos que incluyen transitorios, el resultado fue cero.
3. **Persistencia:** se alarma solo si el residuo sale de la banda **20 ms seguidos**. Esto evita alarmas por picos aislados (es la idea de "k de n").

## Ejecutar
```bash
python simulacion_fallas.py
```

## Ejercicios
1. Diseña un residuo que separe "rodamiento" de "sobrecarga". Pista: compara el par estimado a dos velocidades.
2. Agrega una falla intermitente (sensor que pierde el 30 % de los datos) y mide la detección.
3. Implementa CUSUM sobre `r_e` y compara su tiempo de detección para fallas pequeñas (R +5 %).
4. Haz que el firmware pase a **modo degradado** (control sin sensor de corriente) al detectar esa falla.
