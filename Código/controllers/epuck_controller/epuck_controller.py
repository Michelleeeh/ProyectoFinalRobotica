from controller import Robot
from mapa import cargar_mapa, MAPA_SIMPLE, MAPA_COMPLEJO
from astar import astar

robot = Robot()
TIME_STEP = 64

ENTORNO = "simple"   # "simple" o "complejo"

if ENTORNO == "simple":
    grid, inicio, meta = cargar_mapa(MAPA_SIMPLE)

elif ENTORNO == "complejo":
    grid, inicio, meta = cargar_mapa(MAPA_COMPLEJO)

else:
    raise ValueError("Entorno inválido")

ruta = astar(grid, inicio, meta)

print("\n--- MAPA CARGADO ---")
print("Inicio:", inicio)
print("Meta:", meta)

print("\n--- RUTA ENCONTRADA ---")

if ruta is None:
    print("No existe ruta.")
else:
    print("Longitud:", len(ruta))
    for punto in ruta:
        print(punto)

while robot.step(TIME_STEP) != -1:
    pass
