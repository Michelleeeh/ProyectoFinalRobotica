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
## 7 Resultados obtenidos y métricas de desempeño
El análisis del desempeño del sistema de navegación autónoma híbrido se realizó mediante la evaluación de los datos empíricos recolectados en tiempo real por el controlador. A continuación, se presentan las métricas e interpretaciones cuantitativas y cualitativas de la prueba definitiva ejecutada en el **Entorno Complejo** (grilla de $16 \times 16$ celdas, con un paso de tiempo continuo $dt = 64\text{ ms}$).

### 7.1. Métricas Cuantitativas de Rendimiento Global

Para evaluar objetivamente la eficiencia temporal, cinemática y la robustez del estimador estocástico del e-puck, se consolidaron los siguientes indicadores clave de rendimiento (KPIs) basados en el tiempo de simulación real registrado hasta consolidar la detención en la meta:

| Métrica de Desempeño | Valor Obtenido | Unidad / Detalle |
| :--- | :---: | :--- |
| **Tiempo total hasta la meta** | **123.39** | Segundos de simulación acumulados (`123392 ms`) |
| **Longitud de la ruta planificada ($A^*$)** | 7.25 | Metros teóricos libres de colisión |
| **Longitud aproximada de la trayectoria real**| 6.778 | Metros físicos recorridos en la arena |
| **Velocidad media operacional** | 0.00549 | Metros por segundo ($\text{m/s}$) reales |
| **Desvío (*Drift*) máximo del Giroscopio** | 0.7 | Grados ($^\circ$) de error máximo acumulado |
| **Error residual final de orientación (Kalman)**| 0.011 | Grados ($^\circ$) de error en el punto de destino |
| **Número de colisiones o roces registrados** | 0 | Eventos detectados por impacto físico |
| **Cantidad de waypoints globales procesados** | 30 | Nodos de la ruta principal completados |.11.11.11

### 7.2 Graficos y analisis
A continuacion se realizará diferentes graficos para evaluar el desempeño todo esto basado en los datos obtenidos de **ruta_log**

### 7.2.1 Error de Distancia
Este gráfico muestra cómo cambia la columna error_m a lo largo del tiempo. Es el más fácil de explicar visualmente porque se comporta en "dientes de sierra".Histograma de Velocidad (Comportamiento Adaptativo)
<p align="center">
  <img src="https://github.com/user-attachments/assets/4af8f031-da0b-4867-92b2-6348e8396312" width="550" alt="Gráfico de Error de Distancia" />
</p>

#### Interpretación
Este gráfico representa el pulso de la navegación del robot. Cada pico hacia arriba es el instante exacto donde el e-puck cambia de manera secuencial al siguiente de sus 30 waypoints globales, mientras que la bajada suave y continua muestra la acción del controlador proporcional arrastrando con éxito al vehículo hacia cada objetivo parcial. 

El hecho de que la curva sea limpia, decreciente y libre de perturbaciones demuestra que el sistema nunca osciló ni dudó en su rumbo. Se logró una trayectoria fluida gracias a que la sintonización del umbral dinámico (*lookahead*) evitó frenazos intermitentes o detenciones innecesarias al negociar las esquinas del laberinto.

### 7.2.2 Comportamiento Estocástico de la Orientación (Fusión de Sensores)
Este gráfico pone frente a frente el ángulo medido por el giroscopio crudo (theta_gyro), las ruedas (theta_encoder) y tu filtro estocástico (theta_kalman).
<p align="center">
  <img src="https://github.com/user-attachments/assets/4e97a7e9-5cb3-437e-a73f-61cc3c8ca5cf" width="550" alt="Gráfico de Comparación de Orientación" />
</p>

#### Interpretación
Aquí evaluamos el cerebro matemático del robot. A lo largo de los 123 segundos de viaje, el giroscopio crudo (línea roja) acumula ruido físico por integrar la velocidad en el tiempo, desviándose de la realidad. Por otro lado, las ruedas (línea verde) pueden patinar. La línea azul es nuestro Filtro de Kalman. Al calcular la ganancia óptima basándose en las covarianzas, rescata al robot eliminando la deriva inercial, logrando que al final del trayecto el error residual sea de apenas $0.11∘$, manteniendo al e-puck perfectamente cuadrado con el laberinto.

### 7.2.3 Histograma de Velocidad (Comportamiento Adaptativo)
En lugar de un gráfico XY común, un histograma de la tasa de cambio de posición te permite ver de forma muy didáctica cuántas veces y con qué frecuencia el robot varió su velocidad para proteger la mecánica
<p align="center">
  <img src="https://github.com/user-attachments/assets/d5fa768a-5277-4e1e-8bbd-0b5078438167" width="550" alt="Gráfico de Histograma de Velocidad" />
</p>

#### Interpretación
El histograma de velocidades valida cuantitativamente que la conducción del e-puck no se comportó de forma lineal constante, sino de manera adaptativa. La alta densidad de frecuencias concentrada en el límite superior derecho (cerca de los 0.06 m/s) representa al robot operando de manera eficiente a velocidad crucero en los pasillos rectos despejados.

Por otro lado, la dispersión de barras hacia el extremo izquierdo mapea las desaceleraciones controladas introducidas por la ley de control cinemático. Cuando el error angular incrementaba en las esquinas del laberinto complejo, el algoritmo redujo de forma autónoma el avance lineal para priorizar la rotación pura sobre su eje de simetría. Esto penalizó la velocidad media operacional reduciéndola a 0.0549 m/s, pero aseguró una navegación con 0 colisiones, mitigando los esfuerzos mecánicos por inercia destructiva en los actuadores reales.
---
## 8. Instrucciones para Ejecutar la Simulación (Descarga Directa)

Siga estos pasos para descargar el proyecto de forma manual sin utilizar la terminal:

### 8.1 Requisitos Previos
* **Webots:** Instalado en su última versión estable.
* **Python 3.X:** Configurado en las variables de entorno del sistema (`PATH`).

### 8.2 Descarga del Proyecto
* **Descargar ZIP:** Entre al enlace del repositorio en GitHub (`https://github.com/Michelleeeh/ProyectoFinalRobotica`).
* **Extraer Archivos:** Haga clic en el botón verde **Code** (esquina superior derecha), seleccione **Download ZIP** y descomprima el archivo en cualquier carpeta de su computador.

### 8.3 Ejecución en Webots
* **Cargar el Mundo:** Inicie Webots, diríjase a `File` > `Open World...` y abra el archivo `.wbt` del escenario deseado (Simple o Complejo) dentro de la carpeta descomprimida.
* **Asignar Controlador:** En el panel izquierdo de Webots, expanda las propiedades del robot **e-puck** y verifique que el campo `controller` apunte al script de Python del proyecto.
* **Solución de errores:** Si Webots no detecta el entorno, vaya a `Tools` > `Preferences` > `Python Command` y pegue la ruta del ejecutable de su Python local.
* **Simular:** Presione el botón **Play** o **Fast** en la barra superior para iniciar la navegación autónoma con el Filtro de Kalman. Las salidas de telemetría se desplegarán en la consola inferior en tiempo real.



## 9. Conclusión
El desarrollo de este proyecto permitió evaluar con éxito el desempeño de un **Sistema de Navegación Autónoma Híbrido** en la plataforma robótica diferencial *E-puck*. La integración de las tres capas de software demostró que la navegación precisa en entornos complejos no depende de un solo componente, sino de la sinergia y el trabajo en equipo de todo el sistema:

1. **Planificación Inteligente ($A^*$):** El algoritmo global demostró una alta eficiencia espacial al resolver de forma inmediata un laberinto denso, trazando una ruta óptima de 30 waypoints que garantizó márgenes de seguridad perfectos, logrando una navegación limpia con **0 colisiones**.
2. **Cerebro Estocástico robusto (Filtro de Kalman):** Ante una misión prolongada de más de dos minutos (**123.39 segundos**), el Filtro de Kalman cumplió exitosamente su rol analítico. Logró mitigar la deriva inercial (*drift*) acumulada por el giroscopio, estabilizando la orientación con un error residual final despreciable de apenas **$0.11^\circ$**.
3. **Control Adaptativo y Fluido:** Los gráficos y la telemetría validaron que el acoplamiento entre el controlador proporcional y el umbral dinámico (*lookahead*) permitió un movimiento continuo. Al "recortar las esquinas" de forma parabólica para conservar la energía cinética y frenar de manera autónoma en los virajes cerrados, la trayectoria real ejecutada (**6.77 m**) optimizó la distancia teórica de la grilla (**7.25 m**), protegiendo además los actuadores de desgastes mecánicos.

En conclusión, el sistema híbrido implementado demostró una alta robustez y madurez de ingeniería, resolviendo la transición entre el mundo matemático ideal de las matrices y las restricciones físicas continuas del entorno simulado de Webots de manera sobresaliente.

### 9.1. Limitaciones del Sistema Implementado

1. **Restricción de Movimiento Ortogonal en la Planificación ($A^*$):** La grilla discreta actual limita la expansión de nodos a 4 direcciones cardinales. Esto fuerza al algoritmo a trazar trayectorias con geometría de "escalera" basadas en la distancia Manhattan. Aunque el controlador continuo suaviza estos tramos, la ruta original nace con una penalización de distancia teórica innecesaria.
2. **Dependencia de un Entorno Estático:** El planificador global genera una ruta fija al inicio de la simulación. El sistema carece de un algoritmo de replanificación en tiempo real (*Dynamic Replanning*). Si un obstáculo apareciera de forma imprevista bloqueando por completo un pasillo, el robot dependería únicamente de la capa reactiva local, aumentando el riesgo de quedar atrapado en callejones sin salida locales.
3. **Ausencia de Control de Velocidad Curvilíneo Avanzado:** La ley de control actual reduce la velocidad lineal en función del error angular mediante un esquema proporcional simple. Aunque es estable, no modela de forma exacta las aceleraciones centrípetas ni las restricciones dinámicas del motor, lo que limita la velocidad crucero máxima que la plataforma podría alcanzar de forma segura en las transiciones rectas.



### 9.2. Mejoras Futuras Propuestas

1. **Evolución a Grillas de 8 Direcciones o Mapas Continuos:** Se propone expandir la conectividad del algoritmo $A^*$ a 8 vecindades (incluyendo diagonales mediante distancia Euclidiana o Chebyshev) o implementar algoritmos basados en muestreo continuo como **RRT* (Rapidly-exploring Random Trees)**. Esto permitiría generar rutas globales inherentemente más cortas y fluidas desde la fase de planificación.
2. **Implementación de Ventana Dinámica (DWA - Dynamic Window Approach):** Para resolver la navegación en entornos dinámicos, se sugiere acoplar el planificador global con un generador de trayectorias locales como DWA. Este enfoque evalúa el espacio de velocidades del robot en cada ciclo de control, permitiendo esquivar obstáculos móviles respetando los límites de aceleración y tracción física de los motores.
3. **Control de Seguimiento Mediante Algoritmo Pure Pursuit:** Reemplazar el controlador proporcional simple por una estrategia de **Pure Pursuit** o control por **Modelo Predictivo (MPC)**. Al definir un punto de mira adelante (*lookahead point*) que se desplace continuamente sobre la trayectoria curva, el robot optimizaría el cálculo cinemático inverso, reduciendo a valores cercanos a cero el remanente del error de distancia (`error_m`) en las curvas y permitiendo velocidades operacionales más altas.
