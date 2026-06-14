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
