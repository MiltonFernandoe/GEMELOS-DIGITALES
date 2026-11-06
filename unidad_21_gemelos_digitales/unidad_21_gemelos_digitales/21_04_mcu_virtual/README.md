# 21.4 Representación virtual de un microcontrolador

## ¿Por qué modelar el MCU?
El sistema real no es "motor + algoritmo ideal". Es **motor + firmware corriendo en un MCU con limitaciones**. Si el gemelo ignora esas limitaciones, predice un comportamiento que el sistema real nunca tendrá.

Tres niveles de virtualización del MCU, de menor a mayor fidelidad:

| Nivel | Qué se simula | Herramientas |
|---|---|---|
| **Funcional** (este ejemplo) | Periféricos con sus efectos (ADC, PWM, tiempos, UART) | Python |
| **SIL** (*Software-in-the-Loop*) | El código C real compilado para PC, contra el modelo de planta | gcc + Python o FMU |
| **Emulación de instrucciones** | El binario real (.elf) sobre un procesador emulado | Renode, QEMU, Wokwi, simulador de Proteus |
| **HIL** (*Hardware-in-the-Loop*) | El MCU real, conectado a una planta simulada en tiempo real | dSPACE, Speedgoat, otro MCU rápido |

## A) ADC
- **Resolución:** 1 LSB = Vref / 2ᴺ. Con 3,3 V: 12,9 mV con 8 bits y 0,8 mV con 12 bits.
- **Efecto en el control** (gráfica 1): con 8 bits, cada LSB equivale a 1,17 rad/s. Un escalón de 2 rad/s apenas ocupa dos códigos, y aun así los tres lazos lo siguen: el ruido del ADC funciona como *dithering* y la acción integral del PI lo promedia. El precio aparece en el **mando**: con 8 bits el duty oscila unas 7 veces más que con 12 bits (0,28 % frente a 0,04 %). Eso es más calor en el driver, más ruido audible y más desgaste.
- **Otros efectos a modelar:** ruido en LSB, offset, error de ganancia, tiempo de conversión y retardo de muestreo.

## B) PWM
- El motor ve una tensión conmutada, no un promedio. Su inductancia filtra la corriente, y queda un **rizado**:
  ```
  Δi ≈ V·d·(1−d) / (L·f_pwm)
  ```
- El gemelo coincide con la teoría: ~290 mA a 2 kHz y ~29 mA a 20 kHz.
- Por encima de 20 kHz, el ruido no se escucha. Pero sube la pérdida por conmutación en los MOSFET.
- **Detalle de implementación:** el PWM del gemelo se genera con un **contador entero**, como el timer del MCU. Con aritmética de coma flotante (`t·f % 1`), algunos periodos salían con 1 µs de más y la corriente media derivaba.

## C) Planificación y tiempos de ejecución
El firmware tiene tareas con periodo, duración en ciclos y prioridad:

| Tarea | Periodo | Ciclos | Duración a 72 MHz |
|---|---|---|---|
| Control PI | 1 ms | 15 000 | 0,21 ms |
| Telemetría | 10 ms | 20 000 | 0,28 ms |
| Diagnóstico FFT | 100 ms | 120 000 | **1,67 ms**, más que un tick |

**Resultados** (gráfica 3):
- **72 MHz con la FFT en un bloque:** la FFT desborda el tick, el lazo de control se retrasa hasta ~1,3 ms (**jitter**) y hay *overruns*. Un control que a veces llega tarde se comporta peor de lo que predice el diseño.
- **FFT dividida en 10 partes**, o un MCU a 168 MHz: el problema desaparece.

El gemelo permite **dimensionar el MCU y el firmware antes de fabricar**.

## D) Telemetría UART
- **Trama binaria de 17 bytes:**
  ```
  cabecera AA55 | secuencia | tiempo | ω | i | duty | T | checksum
  ```
  Con números en punto fijo (por ejemplo, corriente en mA en un int16).
- **Ancho de banda:** a 115200 baudios en formato 8N1 se transmiten 11 520 B/s. 1000 tramas/s usan el **148 %**, así que no caben.
- **Checksum XOR:** detecta el ~97–100 % de las tramas corruptas. Para canales ruidosos, conviene usar CRC-16.

## Ejecutar
```bash
python mcu_virtual.py
```

## Ejercicios
1. Implementa CRC-16 CCITT y compara su detección con la del XOR para BER = 1e-2.
2. Agrega a `MCUVirtual` una tarea esporádica (una interrupción de comunicación) y mide el peor caso.
3. Haz el lazo de control con aritmética Q15 y compara con float.
4. Prueba el firmware del módulo 02 del curso anterior en Renode o Wokwi.
