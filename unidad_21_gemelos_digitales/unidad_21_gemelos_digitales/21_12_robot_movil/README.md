# 21.12 Digital Twin de robots móviles

## Qué representa el gemelo de un robot
- **Estado:** pose (x, y, θ), velocidades, batería, temperatura de los motores.
- **Modelo:** cinemática (y dinámica) del robot, sus ruedas, sus sensores.
- **Entorno:** mapa y obstáculos (en robots avanzados, un "gemelo del entorno").
- **Usos:** localización, predicción de trayectoria, detección de colisiones, planificación, autocalibración, simulación de flotas (por ejemplo, robots de almacén) y pruebas de software antes de desplegarlo.

## Cinemática diferencial
```
v = (r_der·ω_der + r_izq·ω_izq) / 2        ω_robot = (r_der·ω_der − r_izq·ω_izq) / base
x' = v·cos θ     y' = v·sin θ     θ' = ω_robot
```
**Odometría:** se integran los incrementos de los encoders. Su gran problema es que **el error crece sin límite**:
- **Errores sistemáticos:** radios distintos, base mal medida. Una diferencia de radios del ~2 % produce ~0,1–0,15 rad de giro parásito por metro.
- **Errores aleatorios:** deslizamiento, irregularidades del suelo.

En el ejemplo, solo con odometría el robot termina **~3 m** fuera de su camino tras recorrer 16 m.

## El gemelo: EKF + autocalibración
**Fusión de sensores:**
- **Odometría** a 50 Hz: precisa a corto plazo y se deriva a largo plazo.
- **Balizas** a 1 Hz, con ±5 cm (UWB, cámara cenital, marcadores ArUco, GPS-RTK en exteriores): no se derivan, pero son ruidosas y lentas.
- El **EKF** combina lo mejor de ambas.

**Lección clave: los errores sistemáticos no son ruido.** El primer EKF trataba la diferencia de radios como ruido aleatorio. Resultado: se volvía **sobreconfiado** (decía ±2 cm y se equivocaba 50 cm).

**Solución:** agregar al estado la **curvatura parásita** `kd` (rad/m) y dejar que el EKF la **aprenda**. El gemelo se **calibra a sí mismo**:

| Escenario | Error al llegar a los puntos | Error máximo |
|---|---|---|
| Solo odometría | 24 → 300 cm (crece) | ~3 m |
| Gemelo con EKF + kd | ~8–11 cm (estable) | ~7 cm |

**Bidireccional:** el gemelo devuelve la pose corregida al robot, que navega con ella. Eso lo convierte en gemelo digital y no en sombra.

## Predicción de trayectoria
El gemelo simula los **próximos 3 s** varias veces (Monte Carlo sobre la pose y los parámetros). El abanico de trayectorias indica **por dónde puede pasar el robot**. Con eso se puede:
- verificar colisiones **antes** de que ocurran;
- reducir la velocidad si el abanico toca un obstáculo;
- coordinar varios robots que comparten pasillos.

## Herramientas reales
| Herramienta | Uso |
|---|---|
| ROS 2 + Gazebo / Ignition | Gemelo con física 3D, sensores simulados (LiDAR, cámaras) |
| `robot_localization` (ROS) | EKF/UKF listo para odometría + IMU + GPS |
| NVIDIA Isaac Sim, Webots, CoppeliaSim | Simulación de alta fidelidad y entrenamiento de políticas |

## Ejecutar
```bash
python gemelo_robot.py      # genera también resultados/4_animacion.gif
```

## Ejercicios
1. Agrega un obstáculo y haz que el robot frene si más del 10 % de las trayectorias predichas lo tocan.
2. Agrega un giroscopio (IMU) como medición de θ'. ¿Cuánto mejora entre balizas?
3. Estima también el error de la base (`base`) como un quinto estado.
4. Simula que las balizas desaparecen 20 s (túnel). ¿Cuánto crece el error con kd ya aprendido frente a sin él?
