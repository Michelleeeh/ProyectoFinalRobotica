"""
epuck_controller.py

Sistema de coordenadas Webots (ENU):
  X → incrementa hacia la DERECHA (Este)
  Y → incrementa hacia ARRIBA (Norte)
  Z → Altura 
"""

import math
import csv
import os

DEBUG = True

# Si DEBUG es True, se importará la función de dibujo de ruta y se usará Supervisor para visualizar la ruta planificada.
if DEBUG:
    from lineadebug import dibujar_ruta_3d
    from controller import Supervisor
    robot = Supervisor()
else:
    from controller import Robot
    robot = Robot()

from mapa import cargar_mapa, MAPA_SIMPLE, MAPA_COMPLEJO
from astar import astar

MAPA_SELECCIONADO = MAPA_SIMPLE 

# ═══════════════════════════════════════════════════════════════════════════════
# PARÁMETROS DEL MAPA Y MUNDO (Arena 4x4m)
# ═══════════════════════════════════════════════════════════════════════════════

TIME_STEP  = 64          
CELL_SIZE  = 0.25      # Cada celda mide 0.25m (16x16 celdas = 4x4 metros)

# Calcula dimensiones y bordes físicos (ORIGEN_X y ORIGEN_Y)
num_filas = len(MAPA_SELECCIONADO)
num_cols  = len(MAPA_SELECCIONADO[0])

# El centro es 0,0. Por lo tanto, el borde es la mitad de la longitud total (teniendo en cuenta que los mapas están hechos de esa forma)
ORIGEN_X = -(num_cols * CELL_SIZE) / 2.0  
ORIGEN_Y =  (num_filas * CELL_SIZE) / 2.0 

WHEEL_RADIUS   = 0.0205  
WHEEL_DISTANCE = 0.0576  
INITIAL_HEADING = 0.0    # 0 radianes apunta hacia +X (Este)

SPEED_BASE        = 3.0   
SPEED_TURN        = 2.0   
KP_ANGULAR        = 2.5   
MAX_SPEED         = 6.28  
ARRIVAL_THRESHOLD = 0.08  
OBSTACLE_THRESHOLD = 150 # Incrementado para evitar falsos positivos por paredes lejanas
TOLERANCIA_ANGULAR = 0.05 # (ej. 0.05 rads son ~2.8 grados)
LOG_FILE = "ruta_log.csv"

# ═══════════════════════════════════════════════════════════════════════════════
# CONVERSIÓN DE COORDENADAS
# ═══════════════════════════════════════════════════════════════════════════════

def celda_a_mundo(fila, col):
    """
    Convierte celdas de la grilla (fila, col) al mundo real (x, y) en el centro de la celda.
    Fila 0 inicia en el borde superior (+2.0 Y).
    Col 0 inicia en el borde izquierdo (-2.0 X).
    """
    x = ORIGEN_X + col  * CELL_SIZE + (CELL_SIZE / 2.0)
    y = ORIGEN_Y - fila * CELL_SIZE - (CELL_SIZE / 2.0) # Restamos porque fila aumenta hacia abajo
    return x, y

# ═══════════════════════════════════════════════════════════════════════════════
# INICIALIZACIÓN DE DISPOSITIVOS
# ═══════════════════════════════════════════════════════════════════════════════

motor_izq = robot.getDevice("left wheel motor")
motor_der = robot.getDevice("right wheel motor")
motor_izq.setPosition(float("inf"))   
motor_der.setPosition(float("inf"))
motor_izq.setVelocity(0.0)
motor_der.setVelocity(0.0)

enc_izq = robot.getDevice("left wheel sensor")
enc_der = robot.getDevice("right wheel sensor")
enc_izq.enable(TIME_STEP)
enc_der.enable(TIME_STEP)

sensores = []
for i in range(8):
    s = robot.getDevice(f"ps{i}")
    s.enable(TIME_STEP)
    sensores.append(s)

robot.step(TIME_STEP)

# ═══════════════════════════════════════════════════════════════════════════════
# PLANIFICACIÓN GLOBAL (A*)
# ═══════════════════════════════════════════════════════════════════════════════

INICIO_GRID = None
META_GRID = None

for f, fila_str in enumerate(MAPA_SELECCIONADO):
    for c, char in enumerate(fila_str):
        if char == 'S':
            INICIO_GRID = (f, c)
        elif char == 'M':
            META_GRID = (f, c)

# Validar que el mapa efectivamente tenía una 'S' y una 'M'
if INICIO_GRID is None or META_GRID is None:
    print(" Error: No se encontró 'S' (Inicio) o 'M' (Meta) en MAPA_SELECCIONADO.")
    exit(1)

grid, _, _ = cargar_mapa(MAPA_SELECCIONADO)

ruta = astar(grid, INICIO_GRID, META_GRID)

print("\n═══════════════════════════════")
print("  NAVEGACIÓN E-PUCK con A*")
print("═══════════════════════════════")
print(f"Inicio (grid) : {INICIO_GRID}  →  mundo {celda_a_mundo(*INICIO_GRID)}")
print(f"Meta   (grid) : {META_GRID}    →  mundo {celda_a_mundo(*META_GRID)}")

if ruta is None:
    print("No existe ruta entre inicio y meta. Revisa la grilla de obstáculos.")
    exit(1)

print(f"Ruta generada : {len(ruta)} nodos")

waypoints = [celda_a_mundo(f, c) for f, c in ruta]

if DEBUG:
    # Asegúrate de que la función de debug soporte X, Y y Z=0 si estás usando ENU
    dibujar_ruta_3d(robot, waypoints) 

# ═══════════════════════════════════════════════════════════════════════════════
# ODOMETRÍA (Matemática Estándar X, Y)
# ═══════════════════════════════════════════════════════════════════════════════

class Odometria:
    def __init__(self, x0, y0, theta0=INITIAL_HEADING):
        self.x     = x0
        self.y     = y0
        self.theta = theta0
        self._prev_izq = None
        self._prev_der = None

    def actualizar(self, enc_izq_val, enc_der_val):
        if self._prev_izq is None:
            self._prev_izq = enc_izq_val
            self._prev_der = enc_der_val
            return

        d_izq = (enc_izq_val - self._prev_izq) * WHEEL_RADIUS
        d_der = (enc_der_val - self._prev_der) * WHEEL_RADIUS

        self._prev_izq = enc_izq_val
        self._prev_der = enc_der_val

        d_centro = (d_der + d_izq) / 2.0
        d_theta  = (d_der - d_izq) / WHEEL_DISTANCE

        mid_theta  = self.theta + d_theta / 2.0
        
        self.x    += d_centro * math.cos(mid_theta)
        self.y    += d_centro * math.sin(mid_theta) 
        
        # Normalizar theta
        self.theta = math.atan2(math.sin(self.theta + d_theta),
                                math.cos(self.theta + d_theta))

    @property
    def pos(self):
        return self.x, self.y

x0, y0 = celda_a_mundo(*INICIO_GRID)
odo = Odometria(x0=x0, y0=y0, theta0=INITIAL_HEADING)

# ═══════════════════════════════════════════════════════════════════════════════
# UTILIDADES DE MOTOR Y MATEMÁTICAS
# ═══════════════════════════════════════════════════════════════════════════════

def set_velocidades(vel_izq, vel_der):
    motor_izq.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, vel_izq)))
    motor_der.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, vel_der)))

def detener():
    set_velocidades(0.0, 0.0)

def distancia(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

def angulo_hacia(xr, yr, theta, xd, yd):
    """Error angular desde el heading actual hasta el waypoint objetivo."""
    objetivo = math.atan2(yd - yr, xd - xr)
    error    = objetivo - theta
    return math.atan2(math.sin(error), math.cos(error))

# ═══════════════════════════════════════════════════════════════════════════════
# SENSORES Y EVASIÓN
# ═══════════════════════════════════════════════════════════════════════════════

SENSORES_FRONTALES = [0, 1, 6, 7]   
SENSORES_DERECHA   = [1, 2]          
SENSORES_IZQUIERDA = [5, 6]          

def leer_sensores():
    return [s.getValue() for s in sensores]

def hay_obstaculo_frontal(lec):
    return any(lec[i] > OBSTACLE_THRESHOLD for i in SENSORES_FRONTALES)

log_rows = []

def registrar(t_sim, x, y, theta, wp_idx, error_dist, evadiendo):
    log_rows.append({
        "t_sim_ms"  : t_sim,
        "x"         : round(x, 4),
        "y"         : round(y, 4),
        "theta_deg" : round(math.degrees(theta), 2),
        "waypoint"  : wp_idx,
        "error_m"   : round(error_dist, 4),
        "evadiendo" : int(evadiendo),
    })

def guardar_log():
    with open(LOG_FILE, "w", newline="") as f:
        campos = ["t_sim_ms", "x", "y", "theta_deg", "waypoint", "error_m", "evadiendo"]
        w = csv.DictWriter(f, fieldnames=campos)
        w.writeheader()
        w.writerows(log_rows)
    print(f"\n📄 Log guardado en: {os.path.abspath(LOG_FILE)}")

# ═══════════════════════════════════════════════════════════════════════════════
# BUCLE DE CONTROL PRINCIPAL
# ═══════════════════════════════════════════════════════════════════════════════

wp_idx        = 1      
t_sim         = 0      
evadiendo     = False
dir_evasion   = 1 # 1 para izquierda, -1 para derecha

print("\n  Iniciando navegación…\n")

while robot.step(TIME_STEP) != -1:
    t_sim += TIME_STEP

    odo.actualizar(enc_izq.getValue(), enc_der.getValue())
    xr, yr = odo.pos
    theta  = odo.theta
    lecturas = leer_sensores()

    if wp_idx >= len(waypoints):
        detener()
        print("✅  META ALCANZADA.")
        break

    wx, wy = waypoints[wp_idx]
    dist_wp = distancia(xr, yr, wx, wy)

    # Comprobación de llegada al waypoint
    if dist_wp < ARRIVAL_THRESHOLD:
        print(f"   WP {wp_idx:>3}/{len(waypoints)-1}  ({wx:+.3f}, {wy:+.3f})  alcanzado")
        wp_idx   += 1
        evadiendo = False
        continue

    registrar(t_sim, xr, yr, theta, wp_idx, dist_wp, evadiendo)

    # Comportamiento Reactivo: Evasión de Obstáculos
    # Se puede mejorar, no ha sido el enfoque por ahora
    if hay_obstaculo_frontal(lecturas):
        if not evadiendo:
            evadiendo = True
            print(f"⚠️  Obstáculo detectado — t={t_sim} ms  pos=({xr:+.3f}, {yr:+.3f})")
            # Decidir dirección de giro basado en el lado más congestionado
            izq_val = sum(lecturas[5:7])
            der_val = sum(lecturas[1:3])
            dir_evasion = -1 if izq_val < der_val else 1
            
        # Rotar sobre su propio eje hasta que el frente esté libre
        set_velocidades(-SPEED_TURN * dir_evasion, SPEED_TURN * dir_evasion)
        continue
    else:
        # Si no hay nada al frente, desactivamos evasión
        evadiendo = False

    # Seguimos el waypoint
    error_ang  = angulo_hacia(xr, yr, theta, wx, wy)
    
    if abs(error_ang) > TOLERANCIA_ANGULAR:
        velocidad_base_actual = 0.0  # Frena por completo y rota sobre su propio eje
    else:
        velocidad_base_actual = SPEED_BASE
        
    correccion = KP_ANGULAR * error_ang

    set_velocidades(velocidad_base_actual - correccion,
                    velocidad_base_actual + correccion)

    

detener()
guardar_log()