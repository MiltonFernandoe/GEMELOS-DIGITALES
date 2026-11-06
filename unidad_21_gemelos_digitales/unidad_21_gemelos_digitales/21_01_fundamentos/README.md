# 21.1 Fundamentos de gemelos digitales

## ¿Qué es un gemelo digital?
Un **gemelo digital** (*Digital Twin*) es una representación virtual de un objeto físico concreto, con tres características:

1. **Corresponde a una unidad específica**, no a un modelo genérico. Es *este* motor, con número de serie 1234 y su historial.
2. **Está conectado a ese objeto** y se actualiza con sus datos reales, de forma continua o periódica.
3. **Se usa para decidir y actuar**: predecir, optimizar, diagnosticar o controlar.

> Un modelo de simulación responde "¿cómo se comporta *un* motor como este?".
> Un gemelo digital responde "¿cómo está y qué le va a pasar a *este* motor, ahora?".

## Los tres niveles de integración
Según la clasificación de Kritzinger et al. (2018), se distinguen por cómo fluyen los datos:

| Nivel | Físico → Virtual | Virtual → Físico | Ejemplo |
|---|---|---|---|
| **Modelo digital** | manual o ninguno | manual o ninguno | Simulación en MATLAB o Python del diseño |
| **Sombra digital** | **automático** | manual | Tablero que muestra el estado del motor en vivo |
| **Gemelo digital** | **automático** | **automático** | El sistema baja la velocidad al predecir sobrecalentamiento |

## El ejemplo
Un motor mueve una bomba durante 10 minutos y la demanda aumenta dos veces sin aviso:

- **Modelo digital:** sigue suponiendo la carga de diseño. Se equivoca en ~31 °C de media y no ve que el motor supera el límite del aislamiento (130 °C).
- **Sombra digital:** recibe temperatura, corriente y velocidad cada 5 s. Estima la carga con un **sensor virtual** (`par = Kt·i − b·ω − Tc`) y refleja la realidad con un error de ~0,4 °C. Pero solo observa: el motor llega a ~146 °C.
- **Gemelo digital:** cada 5 s **predice la temperatura a 60 s** con su modelo sincronizado. Si la predicción supera 110 °C, reduce la tensión (*derating*). El motor se mantiene en ~112 °C y la bomba sigue funcionando a menor caudal.

El *derating* funciona porque la carga es tipo bomba (par ∝ ω²): menos velocidad significa menos par, menos corriente y menos calor. Con una carga de par constante, bajar la tensión **no** enfriaría el motor. Entender la física del proceso es lo que permite elegir una acción que sí sirve.

## Componentes de todo gemelo digital
1. **Entidad física:** el activo, con sus sensores y actuadores.
2. **Entidad virtual:** modelos físicos, basados en datos o híbridos.
3. **Conexión de datos:** telemetría hacia arriba, comandos hacia abajo.
4. **Servicios:** visualización, predicción, optimización, alarmas.
5. **Datos históricos:** el ciclo de vida del activo.

## ¿Por qué en sistemas embebidos?
- El MCU es la **puerta de entrada** de los datos al gemelo: muestrea, filtra, comprime y envía.
- El gemelo puede **probar firmware antes de flashearlo** (HIL/SIL, puesta en marcha virtual).
- Parte del gemelo puede **vivir dentro del MCU**: un modelo reducido que corre en el *edge* para decidir rápido sin depender de la red.

## Beneficios y límites
| Beneficios | Límites y costos |
|---|---|
| Ver magnitudes sin sensor (sensores virtuales) | El modelo nunca es perfecto: hay que validarlo |
| Predecir antes de que ocurra | Requiere conectividad, datos de calidad y sincronización |
| Probar "¿qué pasaría si…?" sin riesgo | Mantenerlo actualizado tiene costo |
| Mantenimiento basado en condición | Seguridad: un canal bidireccional es una superficie de ataque |

## Ejecutar
```bash
python fundamentos.py
```

## Ejercicios
1. Cambia el umbral de *derating* a 120 °C. ¿Cuánto margen de seguridad queda?
2. Haz que la sombra digital reciba datos cada 30 s en lugar de cada 5 s. ¿Cuánto empeora?
3. Cambia la carga a par constante y observa cómo el *derating* deja de funcionar. Propón otra acción.
