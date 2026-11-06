# 21.13 Digital Twin de sistemas industriales

## Contexto
En la industria de procesos (agua, alimentos, química, minería, energía), los gemelos se usan para:

1. **Puesta en marcha virtual** (*virtual commissioning*): probar la lógica del PLC y los lazos de control contra el gemelo **antes** de arrancar la planta.
2. **Monitoreo y detección** de fugas, obstrucciones, degradación de bombas y válvulas.
3. **Análisis "¿qué pasa si…?"** para la operación y la seguridad.
4. **Optimización**: energía, producción, calidad.
5. **Capacitación** de operadores con un simulador realista.

Las piezas típicas son: PLC o MCU industrial, SCADA, OPC UA o Modbus, historizador y modelos de proceso. Para el intercambio de modelos se usan estándares como FMI/FMU (*Functional Mock-up*), y para describir los activos, AAS (*Asset Administration Shell*).

## La planta del ejemplo
```
bomba ──► [Tanque 1] ──válvula──► [Tanque 2] ──► consumo (variable en el turno)
          A1·dh1/dt = k_b·u − c12·√h1 − fuga·√h1
          A2·dh2/dt = c12·√h1 − c2·a·√h2           (ley de Torricelli)
```
Un PI en el PLC regula el nivel del tanque 2 moviendo la bomba. Los tanques tienen 3 m² de sección y la dinámica es de decenas de minutos.

## A) Puesta en marcha virtual
Se prueban tres sintonías **solo en el gemelo**:

| Sintonía | Error medio | Desgaste de la bomba (Σ\|Δu\|) |
|---|---|---|
| Agresiva | ~3,7 cm | ~420 |
| **Moderada** | ~4,7 cm | **~150** |
| Lenta | ~9 cm | ~60 |

La agresiva casi no mejora el error y **triplica el desgaste** de la bomba, así que se elige la moderada. Al aplicarla en la planta real (con parámetros distintos), el comportamiento coincide con el del gemelo.

Así se evitan días de ajuste en planta y los riesgos de rebalse.

## B) Detección de fugas por balance de masa
El controlador **compensa** la fuga: los niveles se ven normales. Pero el gemelo hace el balance:
```
fuga ≈ (lo que entra) − (lo que sale hacia el tanque 2) − (lo que se acumula)
     = k_b·u − c12·√h1 − A1·dh1/dt
```

**Lección clave: primero calibrar el gemelo.** Con los parámetros de catálogo, el balance "ve" una fuga inexistente el ~27 % del tiempo. Calibrando `k_b` y `c12` con datos normales (mínimos cuadrados), la fuga de ~1 L/s se detecta a los **~17 min**, sin falsas alarmas.

**Detalles prácticos:**
- **Filtros causales.** En línea solo se puede usar el pasado. La primera versión, con promedios centrados, "detectaba" la fuga antes de que ocurriera: un error típico al pasar de análisis fuera de línea a tiempo real.
- **Compromiso detección-ruido.** Promediar 15 minutos baja el ruido y permite detectar fugas pequeñas, a cambio de más demora.

## C) Indicador de autonomía en vivo
Cada 2 minutos el gemelo pregunta: *"si la bomba fallara ahora, ¿cuántos minutos hasta que el tanque 2 baje del mínimo?"*.
- Depende del nivel actual **y** del consumo previsto.
- **Validación:** el gemelo predijo ~21 min y la planta real duró ~22 min tras una falla real de la bomba.
- Es un indicador de **riesgo operativo** que ningún sensor da directamente.

## Ejecutar
```bash
python gemelo_industrial.py
```

## Ejercicios
1. Agrega una segunda bomba de respaldo y usa el gemelo para decidir cuándo arrancarla.
2. Simula una obstrucción parcial de la válvula intermedia (c12 baja un 30 %) y diseña su detector.
3. Optimiza el horario de bombeo para usar energía en las horas baratas sin bajar del mínimo.
4. Exporta el modelo de los tanques como FMU con `pythonfmu` y ábrelo en OpenModelica.
