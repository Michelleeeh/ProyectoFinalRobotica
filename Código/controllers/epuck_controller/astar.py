import heapq


def heuristica(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def vecinos(grid, nodo):

    filas = len(grid)
    columnas = len(grid[0])

    movimientos = [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1)
    ]

    resultado = []

    for dr, dc in movimientos:

        nr = nodo[0] + dr
        nc = nodo[1] + dc

        if 0 <= nr < filas and 0 <= nc < columnas:

            if grid[nr][nc] == 0:
                resultado.append((nr, nc))

    return resultado


def reconstruir_camino(vino_de, actual):

    ruta = [actual]

    while actual in vino_de:
        actual = vino_de[actual]
        ruta.append(actual)

    ruta.reverse()

    return ruta


def astar(grid, inicio, meta):

    abiertos = []

    heapq.heappush(abiertos, (0, inicio))

    vino_de = {}

    g_cost = {
        inicio: 0
    }

    while abiertos:

        _, actual = heapq.heappop(abiertos)

        if actual == meta:
            return reconstruir_camino(vino_de, actual)

        for vecino in vecinos(grid, actual):

            nuevo_costo = g_cost[actual] + 1

            if vecino not in g_cost or nuevo_costo < g_cost[vecino]:

                g_cost[vecino] = nuevo_costo

                prioridad = (
                    nuevo_costo +
                    heuristica(vecino, meta)
                )

                heapq.heappush(
                    abiertos,
                    (prioridad, vecino)
                )

                vino_de[vecino] = actual

    return None