import sys
from lib.dibrobot import dibrobot
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    coordenadas = list()
    with open(sys.argv[1], 'r') as coordenadas:
        print("Leyendo archivo de coordenadas")
        coordenadas = coordenadas.read().splitlines()
        # save last element
        ultima_coordenada = coordenadas[-1]
        coord_def = []

        coord_def = coordenadas[::10]
        coord_def.append(ultima_coordenada)
        
        for i in range(len(coord_def)):
            if coord_def[i][0] != "#":    
                coord_def[i] = coord_def[i].split(",")
                coord_def[i][0] = float(coord_def[i][0])
                coord_def[i][1] = float(coord_def[i][1])
                coord_def[i][2] = float(coord_def[i][2])
                # delete next elements
                del coord_def[i][3:]


    if len(sys.argv) > 3 :
        for coordenada in coord_def:
            plt.plot(coordenada[0],coordenada[1],"o")
    else:
        for coordenada in coord_def:
            if coordenada[0] != "#":
                dibrobot([
                    coordenada[0],
                    coordenada[1],
                    coordenada[2]
                ],'','p')

    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.title('Simulación del Movimiento del Robot')
    plt.xlabel('X')
    plt.ylabel('Y')
    
    plt.show()