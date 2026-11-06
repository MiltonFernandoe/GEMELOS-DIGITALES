# 21.2 Arquitectura de un Digital Twin

## Arquitectura en capas
Los estándares (ISO 23247 para manufactura, RAMI 4.0, la arquitectura del Digital Twin Consortium) coinciden en una estructura por capas:

| Capa | Función | Tecnologías típicas |
|---|---|---|
| **1. Física** | Activo, sensores, actuadores | Motor, IMU, termopar, encoder, válvula |
| **2. Edge** | Adquisición, filtrado, compresión, control local, seguridad | MCU (STM32, ESP32), PLC, gateway |
| **3. Comunicación** | Transporte de datos y comandos | MQTT, OPC UA, CoAP, Modbus, CAN, LoRaWAN, TLS |
| **4. Datos** | Almacenamiento histórico y en vivo | Bases de series de tiempo (InfluxDB, TimescaleDB), data lake |
| **5. Modelo** | El gemelo propiamente dicho: física + datos + ML, sincronización | Python, Modelica/FMU, Simulink, PyTorch |
| **6. Servicios** | Lo que el usuario consume | Tableros (Grafana), alarmas, órdenes de mantenimiento, API |

**Funciones transversales:** seguridad, identidad del activo (cada gemelo pertenece a *un* número de serie), versionado de modelos y sincronización de relojes (NTP/PTP).

## Decisiones de diseño clave
### ¿Dónde se calcula cada cosa?
- **En el edge (MCU):** lo que necesita respuesta en milisegundos, como el control, las protecciones o el filtrado. También la reducción de datos.
- **En la nube o servidor:** modelos pesados, historial de flota, ML de largo plazo.
- **Regla práctica:** *si la decisión no puede esperar a la red, va en el edge.*

### Reducción de datos en el edge
El script lo mide:

| Etapa | Datos |
|---|---|
| Crudo a 1 kHz | 4000 B/s |
| Decimado a 50 Hz (promedio de 20 muestras) | 850 B/s |
| + **Banda muerta** (enviar solo si cambia más de un umbral, más un "latido" cada 1 s) | ~200 B/s |

Son unas **19 veces menos datos**. Esto ahorra radio, batería y almacenamiento. La gráfica 4 muestra el precio: cuando la señal está estable llegan pocas muestras.

### Presupuesto de latencia
Cada capa agrega retardo: el muestreo y promedio (~10 ms), el procesamiento en el MCU, la red (~40–100 ms en WiFi o celular), la ingesta en el broker y la base de datos, y el modelo. Hay que diseñar **con el percentil 95 o 99**, no con la media.

## El ejemplo
El script implementa las 6 capas en ~100 líneas: el motor, el MCU con decimación y banda muerta, la red con pérdidas, un almacén circular, el gemelo y un servicio de alarmas. Mide el volumen de datos y la latencia en cada salto.

## Ejecutar
```bash
python arquitectura.py
```

## Ejercicios
1. Cambia `BANDA_MUERTA` a 0,5 y a 10 rad/s. Grafica bytes/s contra el error de reconstrucción.
2. Agrega una capa de "compresión": envía la diferencia con la trama anterior en int8.
3. Simula una red celular (retardo 150 ms, jitter 80 ms). ¿Sigue siendo útil el gemelo para control?
