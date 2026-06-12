from controller import Robot

from mapa import cargar_mapa
from astar import astar


robot = Robot()

TIME_STEP = 64


grid, inicio, meta = cargar_mapa()

ruta = astar(
    grid,
    inicio,
    meta
)

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