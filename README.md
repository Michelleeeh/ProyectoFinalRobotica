# Proyecto Final de Robotica
## Asignatura: Robótica y Sistemas Autónomos (ICI4150-2)

### Integrantes:
* Alfredo Escobar
* José Mena
* Branco González
* Michelle Hernández
* Javier Nuñez


---

## 1. Ruta escogida
Para resolver la navegación del robot, implementaremos la Línea A: Planificación de ruta mediante una estrategia dividida en dos etapas clave:

* Grilla de Ocupación: Transformaremos el espacio continuo 3D de Webots en una matriz 2D discreta. En esta grilla, cada celda representará una porción del plano real y se clasificará binariamente como espacio libre (0) o zona con obstáculo (1).
* Algoritmo A*: Sobre esta matriz, aplicaremos el algoritmo A* para calcular la ruta más corta y eficiente desde el punto de origen hasta la meta. El algoritmo utilizará una función heurística para asegurar que el robot esquive los obstáculos en tiempo mínimo antes de iniciar su movimiento.

---

## 2. Objetivo del proyecto
El presente trabajo tiene como objetivo diseñar e implementar un sistema de navegación autónoma para un robot móvil diferencial modelo e-puck, desarrollado íntegramente dentro del entorno de simulación Webots.

Este proyecto consolida y articula los conocimientos de robótica móvil adquiridos a lo largo del curso —previamente validados en los laboratorios 1 y 2—, permitiendo evaluar, contrastar y analizar el desempeño cinemático del robot tanto en escenarios controlados (simples) como en entornos dinámicos de alta densidad de obstáculos (complejos).

---
## 3. Descripción del robot
El robot utilizado como ya mencionamos antes de un robot diferencial modelo "e-puck" (mismo modelo implementado en el laboratorio  1 y 2) que cuenta con los siguientes sensores y actuadores:

* **Sensores de distancia frontales:** Nuestro robot implementa sensores de proximidad (ps0 y ps7) que nos permitirá medir la proximidad frontal de los obstaculos.
* **Sensores de distancia laterales:** Nuestro robot implementa un sensor de proximidad izquierdo (ps5) y uno derecho (ps2). Se utilizan con un doble propósito: control proporcional de centrado en pasillos durante la marcha recta y toma de decisiones de evasión ante bloqueos frontales.
* **Motor:** Es nuestro principal actuador enfocado en el movimiento cinematico del robot.
* **Encoders de Rueda:** Sensores de posición angular `left wheel sensor` y `right wheel sensor`, con los que se mide el desplazamiento acumulado de cada rueda en radianes para calcular de manera precisa el avance lineal del robot a través de odometría.
---
## 4. Descripción del entorno
En nuestro proyecto diseñamos dos entornos para realizar nuestras pruebas , siendo uno simple que se caracteriza por tener obstaculos que no deberian de suponer un gran desafio para el robot ,y otro más complejo siendo un mayor reto a comparación del simple.

### 4.1 Entorno simple

<p align="center">
  <img width="369" height="358" alt="entorno simple" src="https://github.com/user-attachments/assets/3e51830b-e164-4e4b-9b10-b24859e9abe2" />
</p>

El escenario de la simulación consiste en un tablero cuadrado cerrado, similar a una arena de pruebas, que tiene las siguientes características visuales y estructurales:

* **El Suelo:** Presenta un diseño de tablero de ajedrez (8x8) con baldosas alternadas de color marrón claro y oscuro. Este patrón ayuda a medir visualmente los movimientos y las distancias del robot.
* **Los Límites:** Toda la arena está rodeada por paredes oscuras que mantienen al robot dentro de la zona de juego de manera segura.
* **Punto de Partida (Cuadrante Verde):** Ubicado en la esquina superior izquierda. Es el cuadro de inicio desde donde el robot arranca su viaje.
* **Punto de Destino (Cuadrante Rojo):** Ubicado en la esquina inferior derecha. Es la meta final a la que el robot debe llegar.
* **Los Obstáculos:** El camino entre el inicio verde y la meta roja está bloqueado por dos tipos de obstáculos estáticos que forman una especie de laberinto:
  * **Cajas Rectangulares:** Bloques alargados de color madera distribuidos por el centro que obligan al robot a cambiar de pasillo.
  * **Postes Cilíndricos:** Columnas circulares de color gris oscuro colocadas cerca de las esquinas de las cajas para hacer el paso más estrecho y desafiante.

### 4.2 Entorno Complejo



<p align="center">
  <img width="450" alt="entorno complejo" src="https://github.com/user-attachments/assets/41ea83f7-2c79-4843-b79a-f2d66172366f" />
</p>

El segundo escenario de simulación presenta un aumento significativo en la densidad de obstáculos y la dificultad del recorrido, manteniendo las siguientes características visuales y estructurales:

* **El Suelo:** Continúa utilizando el diseño de tablero de ajedrez, pero a una escala mucho más detallada (16x16 celdas), alternando baldosas de color marrón claro y oscuro. Este aumento de cuadrantes permite medir con mayor precisión los desplazamientos del robot en pasillos estrechos.
* **Los Límites:** Al igual que el entorno anterior, la arena se encuentra completamente cerrada por paredes oscuras que delimitan de forma segura el área operativa del juego.
* **Punto de Partida (Cuadrante Verde):** Ubicado en la esquina superior izquierda. Sigue siendo la zona de inicio desde donde el robot arranca su trayectoria.
* **Punto de Destino (Cuadrante Rojo):** Ubicado en la esquina inferior derecha. Representa la meta final que el robot debe alcanzar tras resolver el recorrido.
* **Los Obstáculos (El Laberinto):** En este escenario, el camino directo está completamente bloqueado por una red mucho más densa y compacta de estructuras estáticas:
  * **Muros Alargados y Cajas:** Bloques de color madera dispuestos estratégicamente para formar callejones sin salida, pasillos largos y esquinas cerradas. Esto fuerza al robot a realizar múltiples cambios de dirección en zigzag.
  * **Postes Cilíndricos:** Columnas circulares de color gris oscuro distribuidas a lo largo del mapa que bloquean las intersecciones y reducen el espacio transitable en los giros, elevando el nivel de exigencia para evitar colisiones.

A continuación se adjunta un link como ejemplo de ejecución de nuestro controlador en base al mapa anteriormente mencionado

<a href="https://www.youtube.com/watch?v=P30xHKlR0dE" target="_blank">
<img src="https://img.youtube.com/vi/P30xHKlR0dE/mqdefault.jpg"
alt="Watch the video" width="560" height="315" />
</a>


---
## 5 Algoritmo implementado

Para lograr la navegación autónoma del robot móvil diferencial E-puck, el sistema de control integra tres capas de software que operan de forma síncrona: un planificador estático que calcula la trayectoria global, un estimador estocástico que estabiliza la orientación del vehículo y un controlador cinemático acoplado a una capa de arbitraje reactiva local.

### 5.1. Capa de Planificación Global (Algoritmo A*)
El comportamiento del robot inicia antes del movimiento físico mediante una fase de abstracción espacial. El controlador invoca al algoritmo de búsqueda heurística $A^*$ (`astar.py`), el cual procesa la grilla de ocupación discreta definida en `mapa.py`. En este espacio, las celdas libres son indexadas con valor `0` y los obstáculos estáticos (muros de madera y postes cilíndricos) con valor `1`.

Para explorar la matriz de forma eficiente y evitar callejones sin salida, el algoritmo expande sus nodos en 4 direcciones ortogonales evaluando la función de costo total $f(n) = g(n) + h(n)$. El costo real acumulado $g(n)$ incrementa de forma unitaria con cada celda visitada, mientras que el costo estimado $h(n)$ se gobierna mediante la **distancia Manhattan**, una heurística óptima para grillas de movimiento restringido que calcula la distancia ortogonal directa hacia la meta:
$$h(a, b) = |a_x - b_x| + |a_y - b_y|$$

Una vez obtenida la secuencia óptima de celdas desde el origen (Zona Verde) hasta el destino (Zona Roja), la función `celda_a_mundo` mapea los índices discretos de la matriz y los transforma en coordenadas cartesianas continuas $(X, Y)$ expresadas en metros, guardándolas cronológicamente en una lista de puntos de referencia o `waypoints`.

---

### 5.2. Capa de Estimación de Estado y Fusión Sensorial (Filtro de Kalman)
Con el robot en marcha, el controlador requiere conocer con alta fidelidad la orientación angular ($\theta$) del vehículo sobre el plano de navegación. Debido a que la odometría pura basada en los *encoders* acumula errores por deslizamiento mecánico en el suelo, y el giroscopio introduce un desvío progresivo (*drift inercial*) al integrar el ruido de alta frecuencia en el tiempo, se diseñó un lazo cerrado de **Fusión Sensorial mediante un Filtro de Kalman Discreto**:

1. **Predicción del Estado:** Utilizando el paso de tiempo del controlador ($dt = 64\text{ ms}$), se integra la velocidad angular del eje Z medida por el giroscopio para proyectar la orientación teórica del robot. En este paso, la incertidumbre del sistema aumenta de acuerdo con la covarianza del ruido del proceso ($P = P + Q$).
2. **Actualización por Medición:** De forma paralela, se calcula la orientación instantánea derivada del modelo cinemático directo de las ruedas utilizando la lectura de impulsos de los *encoders*.
3. **Fusión Óptima:** El algoritmo calcula dinámicamente la **Ganancia de Kalman ($K$)**, un factor de ponderación estadística que determina el nivel de confianza de la predicción frente a la medición basándose en las covarianzas asignadas ($Q = 0.01$, $R = 0.05$). Al multiplicar el error de innovación por esta ganancia, se corrige el estado obteniendo $\theta_{\text{kalman}}$.

Este ángulo corregido es refinado finalmente por un filtro complementario (con pesos fijos de $98\%$ giroscopio y $2\%$ odometría), logrando estabilizar la señal y mitigar por completo la deriva inercial. Esto permite al e-puck ejecutar virajes de $90^\circ$ ortogonales sin perder su cuadratura matemática respecto al entorno real de Webots.

---

### 5.3. Lazo de Control Cinemático y Arbitraje de Comportamientos
La ejecución física del movimiento y la toma de decisiones locales se rigen por la combinación de dos comportamientos concurrentes que compiten por el control de las velocidades de los motores:

* **Seguimiento de Trayectoria (Comportamiento Global):** El controlador mide cíclicamente la distancia euclidiana y el error angular respecto al waypoint activo. El rumbo de las ruedas se corrige mediante un **controlador proporcional ($K_p = 2.0$)** directo sobre el error angular. Para garantizar la continuidad del movimiento, el cambio de waypoint se gestiona con un **umbral de llegada dinámico** adaptativo (*Lookahead Time*). Si el robot se desplaza a velocidad crucero, el radio de aceptación del waypoint se expande proporcionalmente a la velocidad lineal, previniendo deceleraciones bruscas u oscilaciones destructivas en las esquinas.
* **Evasión de Obstáculos (Comportamiento Reactivo Local):** Los 8 sensores analógicos de proximidad infrarrojos (`ps`) monitorizan constantemente el entorno cercano. Mientras las lecturas se mantengan por debajo del umbral de seguridad (`OBSTACLE_THRESHOLD = 160`), el robot opera exclusivamente en Modo Global. Si el e-puck se aproxima críticamente a una estructura, la capa reactiva interrumpe el lazo. Por seguridad, la velocidad base decae instantáneamente al $30\%$ y el controlador evalúa el gradiente de fuerzas analógicas de los flancos. En caso de enfrentar un obstáculo simétrico central, el robot aplica el criterio de **"Magnetismo de Meta"**: evalúa el signo del error angular hacia la ruta global de $A^*$ y fuerza el giro de evasión hacia el flanco que lo aproxime más rápido a su destino original, garantizando que el robot esquive el obstáculo local sin peligro de perder la orientación hacia la meta final.
---
## 6 Pseudocodigo

A continuación se mostrará un pseudocodigo que explicará como funciona el movimiento del robot y como los algoritmos mencionados anteriormente se relacionan entre si con lo relacionado con la navegación del robot
```
ALGORITMO Control del modelo E-PUCk
INICIO

    // 1. CONFIGURACIÓN DE PARÁMETROS Y CONSTANTES DE INGENIERÍA
    Definir TIME_STEP = 64 ms
    Definir CELL_SIZE = 0.25 m               // Cambia a 0.125 m en Entorno Complejo
    Definir MAX_SPEED = 6.28 rad/s
    Definir SPEED_BASE = 3.0, KP_ANGULAR = 2.0
    Definir OBSTACLE_THRESHOLD = 160
    Definir ARRIVAL_THRESHOLD = 0.005, LOOKAHEAD_TIME = 0.1
    
    // Inicialización de matrices de covarianza (Filtro de Kalman)
    Definir P = 1.0, Q = 0.01, R = 0.05
    Definir ALPHA = 0.98                    // Ponderación Filtro Complementario

    // Inicialización de variables de estado
    Definir theta_gyro = 0.0
    Definir theta_kalman = 0.0
    Definir wp_idx = 1
    Definir t_sim = 0 ms

    // 2. INICIALIZACIÓN DE COMPONENTES DE HARDWARE
    Inicializar Motores Diferenciales Izquierdo y Derecho
    Configurar Motores en Modo de Velocidad Continua e inicializar en 0.0 rad/s
    Activar Encoders de Ruedas con frecuencia TIME_STEP
    Activar Giroscopio Inercial con frecuencia TIME_STEP
    Activar 8 Sensores Analógicos de Proximidad Infrarrojos (ps0 a ps7)

    // 3. CAPA DE PLANIFICACIÓN GLOBAL (A*)
    Matriz_Ocupacion = Cargar_Matriz_Desde_Mapa(mapa.py)
    Nodo_Origen = Buscar_Nodo_En_Matriz(Matriz_Ocupacion, 'S')
    Nodo_Destino = Buscar_Nodo_En_Matriz(Matriz_Ocupacion, 'M')
    
    Ruta_Discreta = Ejecutar_A_Estrella(Matriz_Ocupacion, Nodo_Origen, Nodo_Destino)
    
    SI Ruta_Discreta ES NULA ENTONCES
        Abortar_Simulacion("Ruta global no encontrada.")
    FIN SI

    // Transformación al plano continuo de Webots
    Waypoints = Convertir_Nodos_A_Coordenadas_Mundo(Ruta_Discreta, CELL_SIZE)
    Dibujar_Linea_Ruta_Supervisor(Waypoints)

    // 4. LAZO CÍCLICO PRINCIPAL DEL CONTROLADOR
    MIENTRAS Simulador_Avanza_Paso(TIME_STEP) DISPONIBLE HACER
        t_sim = t_sim + TIME_STEP

        // --- CAPA DE PERCEPCIÓN Y ESTIMACIÓN DE ESTADO (FUSIÓN SENSORIAL) ---
        Lectura_Encoders = Obtener_Posicion_Angular_Ruedas()
        Vel_Angular_Z = Obtener_Velocidad_Angular_Gyro_Z()
        Proximidad_Infrarroja = Obtener_Valores_Sensores_PS()
        
        // Odometría clásica (Modelo Diferencial Directo)
        xr, yr, theta_encoder = Calcular_Odometria(Lectura_Encoders)

        // Filtro de Kalman - Etapa de Predicción (Giroscopio)
        dt = TIME_STEP / 1000.0
        theta_gyro = Normalizar_Angulo(theta_gyro + (Vel_Angular_Z * Escala_Inercial) * dt)
        theta_kalman = Normalizar_Angulo(theta_kalman + (Vel_Angular_Z * Escala_Inercial) * dt)
        P = P + Q

        // Filtro de Kalman - Etapa de Actualización (Corrección por Encoder)
        Ganancia_Kalman = P / (P + R)
        theta_kalman = Normalizar_Angulo(theta_kalman + Ganancia_Kalman * (theta_encoder - theta_kalman))
        P = (1.0 - Ganancia_Kalman) * P

        // Fusión Estabilizada mediante Filtro Complementario
        theta_filtrado = Normalizar_Angulo(ALPHA * theta_gyro + (1.0 - ALPHA) * theta_encoder)

        // --- GESTIÓN Y ARBITRAJE DE METAS GLOBALES ---
        SI wp_idx >= Longitud(Waypoints) ENTONCES
            Configurar_Velocidad_Actuadores(0.0, 0.0)
            Imprimir_Consola(" Destino Meta alcanzado con éxito.")
            Romper_Lazo
        FIN SI

        wx, wy = Waypoints[wp_idx]
        dist_objetivo = Calcular_Distancia_Euclidiana(xr, yr, wx, wy)
        
        // Cálculo del Horizonte Adaptativo (Umbral de Llegada Dinámico)
        Umbral_Dinamico = ARRIVAL_THRESHOLD + (LOOKAHEAD_TIME * Absoluto(Velocidad_Lineal_Actual))

        SI dist_objetivo < Umbral_Dinamico ENTONCES
            wp_idx = wp_idx + 1      // Conmutación al siguiente waypoint global
            Continuar_Lazo
        FIN SI

        // Error angular calculado a partir de la estimación filtrada de Kalman
        error_angular = Calcular_Error_Orientacion(xr, yr, theta_kalman, wx, wy)

        // --- CAPA DE ARBITRAJE DE COMPORTAMIENTOS DINÁMICOS ---
        Fuerza_Repulsiva_Izq = Proximidad_Infrarroja[5] + Proximidad_Infrarroja[6] + Proximidad_Infrarroja[7]
        Fuerza_Repulsiva_Der = Proximidad_Infrarroja[0] + Proximidad_Infrarroja[1] + Proximidad_Infrarroja[2]
        
        // Detección crítica de proximidad analógica frontal
        SI Proximidad_Infrarroja[0] > OBSTACLE_THRESHOLD O Proximidad_Infrarroja[7] > OBSTACLE_THRESHOLD ENTONCES
            
            // COMPORTAMIENTO REACTIVO LOCAL (Modo Evasión)
            Velocidad_Lineal_Crucero = SPEED_BASE * 0.3          // Reducción preventiva del 70%
            
            SI Absoluto(Fuerza_Repulsiva_Izq - Fuerza_Repulsiva_Der) < 200 ENTONCES
                // Obstáculo alineado al centro: Criterio Magnetismo de Meta de A*
                SI error_angular > 0 ENTONCES
                    Accion_Control_W = SPEED_BASE                  // Viraje izquierdo
                ELSE
                    Accion_Control_W = -SPEED_BASE                 // Viraje derecho
                FIN SI
            ELSIF Fuerza_Repulsiva_Izq > Fuerza_Repulsiva_Der ENTONCES
                Accion_Control_W = -SPEED_BASE                     // Evasión hacia flanco derecho
            ELSE
                Accion_Control_W = SPEED_BASE                      // Evasión hacia flanco izquierdo
            FIN SI
            
        ELSE
            // COMPORTAMIENTO SEGUIDOR GLOBAL (Modo Tracking)
            // Desaceleración suave en función del error de rumbo
            Velocidad_Lineal_Crucero = SPEED_BASE * Maximo(0.0, 1.0 - Absoluto(error_angular))
            Accion_Control_W = KP_ANGULAR * error_angular          // Control Proporcional clásico
        FIN SI

        // --- SALIDA DE SEÑALES PARA ACTUADORES (CINEMÁTICA INVERSA) ---
        Vel_Motor_Izq = Velocidad_Lineal_Crucero - Accion_Control_W
        Vel_Motor_Der = Velocidad_Lineal_Crucero + Accion_Control_W
        
        // Corregir saturación física para proteger los servomotores de pasos
        Vel_Saturada_Izq = Limitar_Rango(Vel_Motor_Izq, -MAX_SPEED, MAX_SPEED)
        Vel_Saturada_Der = Limitar_Rango(Vel_Motor_Der, -MAX_SPEED, MAX_SPEED)
        
        Configurar_Velocidad_Actuadores(Vel_Saturada_Izq, Vel_Saturada_Der)
        
        // Escritura asíncrona de telemetría empírica
        Registrar_Bitacora_CSV(t_sim, xr, yr, theta_encoder, theta_gyro, theta_kalman, wp_idx, dist_objetivo)

    FIN MIENTRAS

    Guardar_Y_Cerrar_Archivo("ruta_log.csv")
FIN ALGORITMO
```
---
