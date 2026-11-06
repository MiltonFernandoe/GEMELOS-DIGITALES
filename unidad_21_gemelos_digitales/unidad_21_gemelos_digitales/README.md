# UNIDAD XXI — Gemelos digitales (Digital Twins) para sistemas embebidos

Unidad práctica con **teoría, simuladores y gráficas** para cada subtema. Además incluye:
- una **librería de simulación propia** (`gemelo/`) que se reutiliza en todos los ejemplos;
- un **simulador interactivo en vivo**;
- un **caso práctico final** de mantenimiento predictivo sobre una flota de motores.

Todo corre en Python, sin hardware y sin descargar datos.

## Instalación
```bash
# Python 3.9 o superior
pip install -r requirements.txt          # numpy, matplotlib, scikit-learn
```

## Cómo usarlo
```bash
python simulador_en_vivo.py       # ventana interactiva: mueve la carga, inyecta fallas y mira al gemelo
python ejecutar_todo.py           # corre los 14 subtemas (~1 min) y guarda todas las gráficas
```
O un subtema a la vez: entra en su carpeta, lee el `README.md` (teoría) y ejecuta el script. Las gráficas se abren en ventanas y además se guardan en `resultados/`.

**Las gráficas ya vienen generadas** en cada carpeta `resultados/`, así que puedes verlas antes de ejecutar nada.

## Contenido

| Subtema | Ejemplo práctico | Resultado destacado |
|---|---|---|
| [21.1 Fundamentos](21_01_fundamentos/) | Modelo vs sombra vs gemelo en un motor que se sobrecalienta | El gemelo predice y reduce la tensión: 112 °C frente a 146 °C |
| [21.2 Arquitectura](21_02_arquitectura/) | Tubería de 6 capas: física → edge → red → datos → modelo → servicios | El edge reduce 19× los datos. Latencia de extremo a extremo de ~75 ms |
| [21.3 Modelado en Python](21_03_modelado/) | Ecuaciones del motor, Euler vs RK4, identificación de parámetros | Ajuste en validación: 82,6 % (catálogo) → 99,7 % (identificado) |
| [21.4 MCU virtual](21_04_mcu_virtual/) | ADC, PWM, planificador de tareas y telemetría UART simulados | Una FFT mal planificada causa 1,3 ms de jitter en el control |
| [21.5 Sensores y actuadores](21_05_sensores_actuadores/) | 6 defectos de sensor, encoder, zona muerta del puente H | La compensación en firmware recupera el desempeño ideal |
| [21.6 Sincronización](21_06_sincronizacion/) | Lazo abierto vs Kalman vs Kalman con carga | Error: 30 → 2 rad/s. Sensor virtual de par de carga |
| [21.7 Datos en tiempo real](21_07_tiempo_real/) | 3 hilos reales: dispositivo, red con pérdidas, gemelo | Compensar la latencia baja el error de 37 a 1,8 rad/s |
| [21.8 Simulación de fallas](21_08_simulacion_fallas/) | 7 fallas inyectadas, residuos y matriz de firmas | Detección en 19–29 ms y cero falsas alarmas |
| [21.9 Predicción](21_09_prediccion/) | "¿Qué pasa si…?" con 400 simulaciones Monte Carlo | Plan A: 32 % de riesgo. Plan B: seguro |
| [21.10 Machine Learning](21_10_machine_learning/) | El gemelo como fábrica de datos + modelo híbrido física-ML | Domain randomization: 67 % → 89 %. Híbrido 13× mejor al extrapolar |
| [21.11 Motor eléctrico](21_11_motor_electrico/) | Gemelo completo con tablero y temperatura virtual del bobinado | T del bobinado con 2,5 °C de error, frente a 10 °C del sensor real |
| [21.12 Robot móvil](21_12_robot_movil/) | EKF + autocalibración + predicción + animación GIF | Error máximo: 3 m (odometría) → 7 cm (gemelo) |
| [21.13 Sistema industrial](21_13_sistema_industrial/) | Tanques: puesta en marcha virtual, fugas, autonomía | Fuga detectada en 17 min. Autonomía predicha 21 min (real 22) |
| [21.14 Mantenimiento predictivo](21_14_mantenimiento_predictivo/) | **Caso final:** 40 motores, salud diaria, RUL, decisiones | ~1 falla frente a 56 del correctivo. Ahorro de ~84 % |

### Ruta sugerida
- **Conceptos:** 21.1 → 21.2
- **Construir el gemelo:** 21.3 → 21.4 → 21.5
- **Conectarlo:** 21.6 → 21.7
- **Usarlo:** 21.8 → 21.9 → 21.10
- **Aplicaciones:** 21.11 → 21.12 → 21.13 → **21.14 (integrador)**

## La librería `gemelo/`
| Módulo | Contenido |
|---|---|
| `motor.py` | Motor DC eléctrico + mecánico + térmico, con R(T) y carga tipo bomba |
| `robot.py` | Robot diferencial, controlador ir-a-punto, odometría |
| `proceso.py` | Dos tanques en cascada con bomba, válvulas, fuga y falla de bomba |
| `sensores.py` | Sensor genérico (ruido, offset, cuantización, retardo, deriva, fallas), encoder, puente H |
| `mcu.py` | ADC, PWM, planificador con jitter y overruns, tramas UART con checksum |
| `red.py` | Canal con retardo, jitter, pérdidas y desorden; buffer de reordenamiento |
| `estimacion.py` | Kalman, EKF, RLS, discretización exacta (ZOH) |

Puedes usarla en tus propios proyectos: `from gemelo.motor import MotorDC`.

## Lecciones transversales
Cada una apareció de verdad al construir los ejemplos y quedó documentada en su README:

1. **Calibrar el gemelo con datos de *esta* unidad antes de usarlo para detectar algo** (21.3, 21.11, 21.13).
2. **Los errores sistemáticos no son ruido:** hay que modelarlos o estimarlos (21.12).
3. **Un sensor atascado es invisible sin excitación** (21.8).
4. **Los umbrales se calibran con operación normal representativa**, incluidos los transitorios (21.8).
5. **En línea solo existe el pasado:** filtros causales (21.13).
6. **El gemelo debe conocer su propia incertidumbre** y mostrarla (21.6, 21.9, 21.14).
7. **No estimes en línea lo que puedes calibrar antes:** colinealidad (21.11).
8. **El modelo de degradación importa tanto como los datos** (21.14).

## Del simulador al sistema real
Reemplaza la clase física (`MotorDC`, `Tanques`, `RobotDiferencial`) por la lectura de datos reales:
- **Puerto serie:** `pyserial`, con la misma trama de `gemelo/mcu.py`.
- **MQTT:** `paho-mqtt`.
- **Modbus/OPC UA:** `pymodbus`, `asyncua`.

El resto del gemelo (Kalman, residuos, predicción, RUL) no cambia.
