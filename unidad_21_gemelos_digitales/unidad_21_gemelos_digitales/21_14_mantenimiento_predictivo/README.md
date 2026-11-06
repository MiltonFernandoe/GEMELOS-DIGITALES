# 21.14 Aplicación en mantenimiento predictivo (caso práctico final)

## Estrategias de mantenimiento
| Estrategia | Cuándo se interviene | Problema |
|---|---|---|
| **Correctivo** | Cuando se rompe | Paradas imprevistas, daños secundarios, urgencias caras |
| **Preventivo** | Por calendario (cada N días) | Se cambian piezas sanas y aun así hay fallas |
| **Predictivo** (CBM/PdM) | Cuando el estado lo indica, **antes** de fallar | Requiere monitoreo y modelos: ahí entra el gemelo |

## Las tres etapas del pronóstico (PHM: *Prognostics and Health Management*)
1. **Monitoreo de condición.** Calcular un **indicador de salud** (HI) del componente.
2. **Diagnóstico.** ¿Hay degradación? ¿De qué tipo? (21.8, 21.10)
3. **Pronóstico.** ¿Cuánto falta para fallar? Es la **vida útil remanente (RUL)**, con incertidumbre.

## El caso práctico
**Escenario:** 40 motores durante 500 días. Los rodamientos se desgastan: la fricción `b` crece exponencialmente desde un día desconocido y a una velocidad distinta en cada motor.

### 1. Indicador de salud desde el gemelo
Cada día, el MCU ejecuta un **ciclo de diagnóstico**: 5 tensiones sin carga. En régimen, `Kt·i = b·ω + Tc`, así que una recta da `b`. Luego:
```
HI = b / b_inicial      (1 = nuevo, 10 = falla)
```
Esto solo es posible porque el gemelo conoce `Kt`, identificado en la puesta en marcha. Sin modelo, `i` y `ω` crudos no dicen nada directo sobre el rodamiento.

### 2. Detección
Se considera que hay degradación si `HI > 1,6` durante 3 días seguidos.

### 3. Pronóstico de RUL
- **Modelo de degradación:** `HI(t) = c + exp(a + λ·t)`.
- **Ajuste:** se busca `c` en una grilla; para cada valor, el problema es lineal en `log(HI − c)`.
- **Extrapolación:** hasta `HI = 10`.
- **Incertidumbre:** *bootstrap* de residuos sobre el modelo completo, que da los percentiles p10, p50 y p90 de la RUL.

**Lección del desarrollo:** una primera versión ajustaba `log(HI − 1)` como recta, el modelo equivocado para el inicio de la degradación. Subestimaba la RUL, daba saltos y reportaba intervalos de ±1 día (sobreconfianza). **El modelo de degradación importa tanto como los datos.**

**Comportamiento esperado** (gráfica 3):
- Al principio la estimación es optimista y muy incierta.
- Con más datos **converge dentro del cono de ±20 %**.

### 4. Decisión
Se programa el cambio cuando la **RUL pesimista (p10)** sea menor que el **plazo** necesario para comprar el repuesto y planificar la parada (21 días).

## Resultados (40 motores, 500 días)
| Estrategia | Fallas | Intervenciones planificadas | Días de parada | Costo |
|---|---|---|---|---|
| Correctivo | 56 | 0 | 168 | ~672 000 |
| Preventivo (cada 150 días) | 5 | 115 | ~73 | ~233 000 |
| **Predictivo (gemelo)** | **~1** | **~59** | **~33** | **~109 000** |

- **Frente al correctivo:** ahorro del ~84 %.
- **Frente al preventivo:** la mitad de intervenciones, porque no se cambian rodamientos sanos, y menos fallas.

Los costos son ilustrativos: falla $12 000 (repuesto + daños + 3 días de parada), cambio planificado $1 500 y monitoreo $150 por motor y año.

## Cómo se lleva esto a una planta real
1. Sensores mínimos: corriente y velocidad (a veces vibración y temperatura) y un MCU por motor o un gateway.
2. Ciclo de diagnóstico automático en momentos sin producción, o extracción del HI de la operación normal (más difícil).
3. Historial por número de serie: es el "gemelo" de cada activo.
4. Validación del modelo de degradación con fallas reales pasadas. Si no hay, hay que empezar a registrarlas.
5. Integración con el sistema de mantenimiento (CMMS) para generar órdenes de trabajo automáticamente.

**Datasets públicos para practicar:** NASA C-MAPSS (turbinas), FEMTO/PRONOSTIA (rodamientos), CWRU (rodamientos con vibración).

## Ejecutar
```bash
python mantenimiento_predictivo.py
```

## Ejercicios
1. Cambia `PLAZO` a 7 y a 45 días. ¿Cómo cambian las fallas y las intervenciones?
2. Usa el p50 en lugar del p10 para decidir. ¿Qué riesgo se asume?
3. Agrega degradación del bobinado (R0 sube) como segundo modo de falla, con su propio HI.
4. Busca el intervalo preventivo óptimo (60–300 días) y compáralo con el predictivo.
5. Aplica el método al dataset C-MAPSS de la NASA.
