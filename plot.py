import sys
from lib.dibrobot import dibrobot
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    coordenadas = list()
    with open(sys.argv[1], 'r') as coordenadas:
        print("Leyendo archivo de coordenadas")
        coordenadas = coordenadas.read().splitlines()

        coordenadas = coordenadas[::10] # TODO: PONER CON LÍMITE, NO DIVISIÓN
        
        # print(coordenadas)
        for i in range(len(coordenadas)):
            if coordenadas[i][0] != "#":    
                coordenadas[i] = coordenadas[i].split(",")
                coordenadas[i][0] = float(coordenadas[i][0])
                coordenadas[i][1] = float(coordenadas[i][1])
                coordenadas[i][2] = float(coordenadas[i][2])
                # delete next elements
                del coordenadas[i][3:]

    # plt.plot(200,200,"D")

    if len(sys.argv) > 3 :
        for coordenada in coordenadas:
            plt.plot(coordenada[0],coordenada[1],"o")
    else:
        for coordenada in coordenadas:
            if coordenada[0] != "#":
                print([
                    coordenada[0],
                    coordenada[1],
                    coordenada[2]
                ])
                dibrobot([
                    coordenada[0],
                    coordenada[1],
                    coordenada[2]
                ],'','p')

    # plt.xlim(-5,5)
    # plt.ylim(-5,5)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.title('Simulación del Movimiento del Robot')
    plt.xlabel('X')
    plt.ylabel('Y')
    
    plt.show()