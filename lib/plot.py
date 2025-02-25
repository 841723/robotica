import sys
from dibrobot import dibrobot
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    coordenadas = list()
    with open(sys.argv[1], 'r') as coordenadas:
        coordenadas = coordenadas.read().splitlines()
        for x in range(len(coordenadas)):
            coordenadas[x] = coordenadas[x].split(",")
            coordenadas[x][0] = float(coordenadas[x][0])
            coordenadas[x][1] = float(coordenadas[x][1])
            coordenadas[x][2] = np.deg2rad(float(coordenadas[x][2]))

    plt.plot(200,200,"D")

    if len(sys.argv) > 3 :
        for z in range(len(coordenadas)):
            plt.plot(coordenadas[z][0],coordenadas[z][1],"o")
    else:
        for z in range(len(coordenadas)):
            dibrobot(coordenadas[z],'','g')

    plt.show()