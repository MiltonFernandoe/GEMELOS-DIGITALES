# 21.7 Integración de datos en tiempo real

## Qué significa "tiempo real" en un gemelo
No significa "rápido": significa **con plazos acotados y conocidos**. Hay dos escalas:

- **Tiempo real duro**, en el MCU: el lazo de control *debe* ejecutarse cada 1 ms (ver 21.4).
- **Tiempo real blando**, en el gemelo: los datos deben llegar y procesarse lo bastante rápido para que el gemelo sea útil. Los retrasos ocasionales se toleran.

## Problemas reales de los datos en vivo
| Problema | Causa | Solución en el gemelo |
|---|---|---|
| **Latencia** | Red, colas, procesamiento | Marcar el tiempo de origen en el dispositivo y **compensar** |
| **Jitter** | Variación de la latencia | Buffer y uso del timestamp de origen, no del de llegada |
| **Pérdidas** | WiFi, LoRa, desbordes | Número de secuencia y detección de huecos; el gemelo predice en el hueco (21.6) |
| **Desorden** | Rutas distintas, reintentos | **Buffer de reordenamiento** por número de secuencia |
| **Relojes distintos** | Cada dispositivo tiene su reloj | Sincronización NTP/PTP o estimar el desfase |
| **Duplicados** | Reintentos de QoS 1 en MQTT | Descartar por número de secuencia |

## Arquitectura del ejemplo: 3 hilos concurrentes
```
DISPOSITIVO (1 kHz, envía a 50 Hz) ─► RED (retardo + jitter + 5 % pérdida) ─► GEMELO
                                                                               │ decodificar (checksum)
                                                                               │ reordenar por secuencia
                                                                               │ compensar latencia
                                                                               └ registrar
```
- Corre **en tiempo real de verdad**: 8 segundos de reloj, con `threading` y `queue`.
- Es el mismo patrón que se usaría con un broker MQTT real (`paho-mqtt`). Para conectarlo a un dispositivo real, se reemplaza el hilo DISPOSITIVO por la lectura de un puerto serie (`pyserial`) o de un tema MQTT.

## Compensación de latencia
Cada trama dice "en el instante t_origen, la velocidad era ω". Cuando llega, ya pasaron 50–300 ms.

- **Sin compensar:** el gemelo muestra el pasado. Con cambios bruscos, se equivoca ~37 rad/s de media.
- **Compensado:** el gemelo arranca su modelo desde ese dato y **simula hasta el presente**, usando el plan de mando conocido. El error baja a ~2 rad/s.

Es la misma idea que usan los videojuegos en red (*dead reckoning*) y los sistemas de control por red.

## Compromiso del buffer de reordenamiento
- **Esperar más** a las tramas atrasadas reduce los huecos, pero **aumenta la edad** de los datos. En el ejemplo, esperar hasta 5 tramas suma ~100 ms.
- **Esperar menos** baja la latencia, pero hay más "huecos". Algunas tramas llegan después de haber sido dadas por perdidas.

## Ejecutar
```bash
python tiempo_real.py        # dura ~8 s
```

## Ejercicios
1. Reemplaza el canal simulado por MQTT real: `pip install paho-mqtt`, con un broker local Mosquitto.
2. Conecta un Arduino o ESP32 que envíe la trama de `gemelo/mcu.py` por el puerto serie y léela con `pyserial`.
3. Cambia el tamaño máximo del buffer (5 → 1, 20) y grafica la edad media contra los huecos.
4. Agrega un desfase de reloj de +300 ms al dispositivo y propone cómo estimarlo.
