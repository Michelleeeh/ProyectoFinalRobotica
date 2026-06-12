MAPA_RAW = [
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
    "0111011010000100",
    "100000000011000M"
]
def cargar_mapa():
    grid = []
    inicio = None
    meta = None

    for fila, linea in enumerate(MAPA_RAW):

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