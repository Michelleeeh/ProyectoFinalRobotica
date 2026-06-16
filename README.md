# Proyecto Final de Robotica
## Asignatura: Robótica y Sistemas Autónomos (ICI4150-2)

### Integrantes:
* Alfredo Escobar
* José Mena
* Branco González
* Michelle Hernández
* Javier


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


### 4.3 Algoritmo implementado
