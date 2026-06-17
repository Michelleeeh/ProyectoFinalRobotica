MAPA_SIMPLE = [
    "S0100100",
    "00001100",
    "11000000",
    "10000011",
    "00011000",
    "00010000",
    "01000110",
    "0110000M"
]


MAPA_COMPLEJO = [
    "S000000011111110",
    "0000000000000000",
    "0011101110111100",
    "1000000000000000",
    "0000000001000000",
    "0011101111011110",
    "0000000000000001",
    "1000010100000000",
    "1111100000011110",
    "1000001110000000",
    "0000000000110000",
    "0110011000000100",
    "0111011010001101",
    "0111011010011100",
    "0111000010000100",
    "100000000011000M"
]
def cargar_mapa(mapa_raw):

    grid = []
    inicio = None
    meta = None

    for fila, linea in enumerate(mapa_raw):

        nueva_fila = []

        for col, c in enumerate(linea):

            if c == 'S':
                inicio = (fila, col)
                nueva_fila.append(0)

            elif c == 'M':
                meta = (fila, col)
                nueva_fila.append(0)

            else:
                nueva_fila.append(int(c))

        grid.append(nueva_fila)

    return grid, inicio, meta

def leerMapa(robot):

    world_name = robot.getCustomData()
    if world_name == "EntornoComplejo":
        MAPA_SELECCIONADO = MAPA_COMPLEJO
    elif world_name == "EntornoSimple":
        MAPA_SELECCIONADO = MAPA_SIMPLE
    else:
        print(f"[WARN] customData desconocido: '{world_name}', usando MAPA_SIMPLE por defecto.")
        MAPA_SELECCIONADO = MAPA_SIMPLE

    print(f"[INFO] Mapa seleccionado desde customData: '{world_name}'")

    return MAPA_SELECCIONADO

def leerInicioMeta(robot, MAPA_SELECCIONADO, CELL_SIZE):
    """
    Lee la posición de los nodos 'bloqueInicio' y 'bloqueMeta' en el mundo
    y los convierte a celdas de la grilla, con tolerancia para imprecisión manual.
    Requiere que el robot sea Supervisor.
    """

    ORIGEN_X = -(len(MAPA_SELECCIONADO[0]) * CELL_SIZE) / 2.0
    ORIGEN_Y =  (len(MAPA_SELECCIONADO)    * CELL_SIZE) / 2.0

    def mundo_a_celda(x, y):
        # Snap al centro de celda más cercano
        col  = int((x - ORIGEN_X) / CELL_SIZE)
        fila = int((ORIGEN_Y - y) / CELL_SIZE)
        return (fila, col)

    try:
        nodo_inicio = robot.getFromDef("bloqueInicio")
        nodo_meta   = robot.getFromDef("bloqueMeta")

        if nodo_inicio is None or nodo_meta is None:
            raise RuntimeError("No se encontraron los nodos DEF bloqueInicio / bloqueMeta")

        pos_inicio = nodo_inicio.getPosition()  # [x, y, z]
        pos_meta   = nodo_meta.getPosition()

        inicio = mundo_a_celda(pos_inicio[0], pos_inicio[1])
        meta   = mundo_a_celda(pos_meta[0],   pos_meta[1])

        print(f"[INFO] bloqueInicio → mundo ({pos_inicio[0]:.3f}, {pos_inicio[1]:.3f}) → celda {inicio}")
        print(f"[INFO] bloqueMeta   → mundo ({pos_meta[0]:.3f},   {pos_meta[1]:.3f}) → celda {meta}")

        return inicio, meta

    except Exception as e:
        print(f"[WARN] No se pudo leer posición de bloques: {e}")
        print("[WARN] Usando S/M del mapa como fallback.")
        return None, None
