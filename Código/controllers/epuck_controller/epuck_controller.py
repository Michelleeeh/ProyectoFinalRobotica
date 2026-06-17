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

from controller import Supervisor
robot = Supervisor()
node = robot.getSelf()

from lineadebug import dibujar_ruta_3d, checkDebug
from mapa import cargar_mapa, leerMapa, leerInicioMeta
from astar import astar

MAPA_SELECCIONADO = leerMapa(robot)
DEBUG = checkDebug(node) 
 
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
KP_ANGULAR        = 2.0   
MAX_SPEED         = 6.28  
ARRIVAL_THRESHOLD = 0.025
OBSTACLE_THRESHOLD = 550 # Incrementado para evitar falsos positivos por paredes lejanas
TOLERANCIA_ANGULAR = 0.05 # (ej. 0.05 rads son ~2.8 grados)
LOOKAHEAD_TIME = 0.1 # Segundos para calcular un umbral dinámico de llegada basado en la velocidad actual
LOOKAHEAD_DISTANCE = 0.05   # metros
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

gyro = robot.getDevice("gyro")
gyro.enable(TIME_STEP)

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
META_GRID   = None

if DEBUG:
    INICIO_GRID, META_GRID = leerInicioMeta(robot, MAPA_SELECCIONADO, CELL_SIZE)
else:
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
    # Dibuja la ruta planificada en el entorno Webots
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

    def actualizar(self, enc_izq_val, enc_der_val, theta_fusionado=None):
        if self._prev_izq is None:
            self._prev_izq = enc_izq_val
            self._prev_der = enc_der_val
            self.vel_izq = 0.0
            self.vel_der = 0.0
            return

        d_izq = (enc_izq_val - self._prev_izq) * WHEEL_RADIUS
        d_der = (enc_der_val - self._prev_der) * WHEEL_RADIUS

        self._prev_izq = enc_izq_val
        self._prev_der = enc_der_val

        dt = TIME_STEP / 1000.0 
        self.vel_izq = d_izq / dt
        self.vel_der = d_der / dt

        d_centro = (d_der + d_izq) / 2.0
        d_theta  = (d_der - d_izq) / WHEEL_DISTANCE
        
        # Mantenemos el theta de los encoders solo para alimentar al filtro de Kalman
        self.theta = math.atan2(math.sin(self.theta + d_theta),
                                math.cos(self.theta + d_theta))

        # EL FIX: Si el loop principal nos pasa el theta del gyro, lo usamos para la posición.
        angulo_uso = theta_fusionado if theta_fusionado is not None else self.theta
        
        self.x    += d_centro * math.cos(angulo_uso)
        self.y    += d_centro * math.sin(angulo_uso)

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

def registrar(t_sim, x, y, theta_encoder, theta_gyro, theta_kalman, wp_idx, error_dist, evadiendo):
    log_rows.append({
        "t_sim_ms"  : t_sim,
        "x"         : round(x, 4),
        "y"         : round(y, 4),
        "theta_encoder": round(math.degrees(theta_encoder), 2),
        "theta_gyro": round(math.degrees(theta_gyro), 2),
        "theta_kalman": round(math.degrees(theta_kalman), 2),
        "waypoint"  : wp_idx,
        "error_m"   : round(error_dist, 4),
        "evadiendo" : int(evadiendo),
    })

def guardar_log():
    with open(LOG_FILE, "w", newline="") as f:
        campos = ["t_sim_ms", "x", "y", "theta_encoder","theta_gyro", "theta_kalman" ,"waypoint", "error_m", "evadiendo"]
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
estado_pivote = False 
theta_gyro = INITIAL_HEADING
theta_kalman = INITIAL_HEADING

P = 1.0
Q = 0.01
R = 0.05

print("\n  Iniciando navegación…\n")

while robot.step(TIME_STEP) != -1:
    t_sim += TIME_STEP

    odo.actualizar(enc_izq.getValue(), enc_der.getValue(), theta_kalman)
    
    dt = TIME_STEP / 1000.0
    
    gyro_values = gyro.getValues()
    
    gyro_scale = 13.315805 / 100000.0
    omega_z = gyro_values[2] * gyro_scale
    
    theta_gyro += omega_z * dt
    
    # Predicción Kalman
    theta_kalman += omega_z * dt
    P += Q
    
    # Normalizar predicción
    theta_kalman = math.atan2(
        math.sin(theta_kalman),
        math.cos(theta_kalman)
    )
    
    theta_gyro = math.atan2(
        math.sin(theta_gyro),
        math.cos(theta_gyro)
    )
    
    xr, yr = odo.pos
    theta = odo.theta
    
    # Corrección Kalman usando encoder
    K = P / (P + R)
    
    theta_kalman = theta_kalman + K * (theta - theta_kalman)
    
    P = (1 - K) * P
    
    # Normalizar resultado final
    theta_kalman = math.atan2(
        math.sin(theta_kalman),
        math.cos(theta_kalman)
    )
    
    ALPHA = 0.98
    
    theta_fusion = math.atan2(
        math.sin(ALPHA * theta_gyro + (1 - ALPHA) * theta),
        math.cos(ALPHA * theta_gyro + (1 - ALPHA) * theta)
    )
    
    lecturas = leer_sensores()
   
    if wp_idx >= len(waypoints):
        detener()
        print("✅  META ALCANZADA.")
        break


    vel_izq_actual = enc_izq.getValue() 
    current_speed = SPEED_BASE * WHEEL_RADIUS if not evadiendo else 0.0 

    # ══════════════════════════════════════════════════════════════
    # ACTUALIZACIÓN DE WAYPOINT (Lookahead)
    # ══════════════════════════════════════════════════════════════
    
    wx, wy = waypoints[wp_idx]
    dist_wp = distancia(xr, yr, wx, wy)
    es_ultimo_wp = (wp_idx == len(waypoints) - 1)

    umbral_actual = 0.03 if es_ultimo_wp else 0.12 

    if dist_wp < umbral_actual:
        if es_ultimo_wp:
            detener()
            print("✅ META ALCANZADA.")
            break
        else:
            print(f"   WP {wp_idx:>3}/{len(waypoints)-1} alcanzado")
            wp_idx += 1
            evadiendo = False
            estado_pivote = False  # Resetear el estado de giro forzado al cambiar de WP
            continue
    registrar(t_sim, xr, yr, theta, theta_gyro, theta_kalman, wp_idx, dist_wp, evadiendo)

    # "Fuerza de atracción" hacia el waypoint para la evasión reactiva
    error_ang = angulo_hacia(xr, yr, theta_kalman, wx, wy)

    # ════════════════
    # EVASIÓN REACTIVA 
    # ════════════════
    """
    Utilizamos una especie de "Magnetismo" hacia el waypoint para decidir la dirección de evasión cuando hay un obstáculo frontal.
    Esto hace que el robot intente evadir en la dirección que lo acerque más al waypoint, en lugar de simplemente girar a la izquierda o derecha
    sin considerar el objetivo final.
    """

    fuerza_izq = lecturas[5] + lecturas[6] + lecturas[7]
    fuerza_der = lecturas[0] + lecturas[1] + lecturas[2]
    
    if hay_obstaculo_frontal(lecturas):
        print(f"⚠️  Obstáculo detectado — t={t_sim} ms  pos=({xr:+.3f}, {yr:+.3f})")
        # Reducir velocidad base para no chocar mientras maniobra
        velocidad_base_actual = SPEED_BASE * 0.3 
        
        # Determinar de qué lado está más cerca el obstáculo
        diferencia_fuerza = fuerza_izq - fuerza_der
        
        # Umbral de 200 para considerar que el obstáculo está "justo al frente"
        if abs(diferencia_fuerza) < 200: 
            # OBSTÁCULO FRONTAL CENTRADO: Usamos el waypoint para decidir el giro
            if error_ang > 0:
                correccion = SPEED_TURN  # El WP está a la izq, evadimos por la izq
            else:
                correccion = -SPEED_TURN # El WP está a la der, evadimos por la der
                
        elif diferencia_fuerza > 0:
            # OBSTÁCULO A LA IZQUIERDA: Girar a la derecha
            correccion = -SPEED_TURN
        else:
            # OBSTÁCULO A LA DERECHA: Girar a la izquierda
            correccion = SPEED_TURN
            
    else:
        # ══════════════════════════════════════════════════════════════
        # NAVEGACIÓN ESTÁNDAR (Lookahead)
        # ══════════════════════════════════════════════════════════════
        
        UMBRAL_PIVOTE = 0.45 
        
        # Bloqueo estricto solo aplicable si estamos persiguiendo el último WP
        if es_ultimo_wp and dist_wp < 0.10:
            estado_pivote = False
            velocidad_base_actual = SPEED_BASE * 0.5  
            factor_distancia = dist_wp / 0.10 
            correccion = KP_ANGULAR * error_ang * factor_distancia
            correccion = max(-SPEED_TURN * 0.4, min(SPEED_TURN * 0.4, correccion))
            
        else:
            # Histéresis normal para el resto de la ruta
            if abs(error_ang) > UMBRAL_PIVOTE:
                estado_pivote = True
            elif abs(error_ang) < 0.15:
                estado_pivote = False
                
            if estado_pivote:
                # Pivote puro si la esquina es demasiado aguda
                velocidad_base_actual = 0.0
                KP_PIVOTE = 2.5
                correccion = max(-SPEED_TURN, min(SPEED_TURN, KP_PIVOTE * error_ang))
            else:
                # Navegación fluida y rápida
                velocidad_base_actual = max(0.0, SPEED_BASE * math.cos(error_ang))
                correccion = KP_ANGULAR * error_ang

    set_velocidades(velocidad_base_actual - correccion,
                    velocidad_base_actual + correccion)

detener()
guardar_log()