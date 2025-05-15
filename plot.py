import sys
from lib.dibrobot import dibrobot
import matplotlib.pyplot as plt
import numpy as np

def escenario(a):
   # Escenario
    if a: 
        plt.vlines(0, 0, 2.8, colors='k',lw=1.5)
        plt.vlines(1.2, 1.2, 2.8, colors='k',lw=1.5)
        plt.vlines(2.8, 0, 2.8, colors='k',lw=1.5)
        plt.hlines(0, 0, 2.8, colors='k',lw=1.5)
        plt.hlines(2.8, 0, 0.4, colors='k',lw=1.5)
        plt.hlines(2.8, 0.8, 1.2, colors='k',lw=1.5)
        plt.hlines(2.8, 1.6, 2.4, colors='k',lw=1.5)
        plt.hlines(1.2, 1.6, 2.4, colors='k',lw=1.5)
    else: 
        plt.vlines(0, 0, 2.8, colors='k',lw=1.5)
        plt.vlines(1.6, 1.2, 2.8, colors='k',lw=1.5)
        plt.vlines(2.8, 0, 2.8, colors='k',lw=1.5)
        plt.hlines(0, 0, 2.8, colors='k',lw=1.5)
        plt.hlines(2.8, 0.4, 1.2, colors='k',lw=1.5)
        plt.hlines(2.8, 1.6, 2.0, colors='k',lw=1.5)
        plt.hlines(2.8, 2.4, 2.8, colors='k',lw=1.5)
        plt.hlines(1.2, 0.4, 1.2, colors='k',lw=1.5)
        
    # Grid
    plt.hlines(0.4, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.hlines(0.8, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.hlines(1.2, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.hlines(1.6, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.hlines(2.0, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.hlines(2.4, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.hlines(2.8, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.hlines(2.8, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.vlines(0.4, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.vlines(0.8, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.vlines(1.2, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.vlines(1.6, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.vlines(2, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.vlines(2.4, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    plt.vlines(2.8, 0, 2.8, colors='k',linestyles="dotted",lw=0.5)
    # # Cuadros
    # plt.vlines(0, 1.4, 1.8, colors='b',lw=4)
    # plt.hlines(0, 0.4, 0.8, colors='b',lw=4)
    # plt.hlines(3.2, 1.8, 2.2, colors='b',lw=4)

    plt.gca().set_aspect('equal', adjustable='box')

if __name__ == "__main__":
    coordenadas = list()
    with open(sys.argv[1], 'r') as coordenadas:
        print("Leyendo archivo de coordenadas")
        coordenadas = coordenadas.read().splitlines()
        # save last element
        ultima_coordenada = coordenadas[-1]
        coord_def = []

        coord_def = coordenadas[::30]
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
                ],'red','p')
                
    escenario(a=True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(False)
    plt.title('Simulación del Movimiento del Robot')
    plt.xlabel('X')
    plt.ylabel('Y')
    
    plt.show()