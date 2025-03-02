import cv2
import numpy as np

def createDetector(minThreshold=10, maxThreshold=200, minArea=200, maxArea=10000, minCircularity=0.1):
    # TODO: CAMBIAR detector con los parámetros conseguidos en get_color_blobs.py
    # Setup default values for SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # These are just examples, tune your own if needed
    # Change thresholds
    params.minThreshold = minThreshold
    params.maxThreshold = maxThreshold

    # Filter by Area
    params.filterByArea = True
    params.minArea = minArea
    params.maxArea = maxArea

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = minCircularity

    # Filter by Color
    params.filterByColor = False
    # not directly color, but intensity on the channel input
    #params.blobColor = 0
    params.filterByConvexity = False
    params.filterByInertia = False
    
    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else:
        detector = cv2.SimpleBlobDetector_create(params)
    return detector


def calcTrackSpeed(x, area, targetX, objectiveTargetSize, v, w):
    # Calcular error de posición
    error_x = targetX - x
    
    # Calcular error de tamaño
    error_size = np.sqrt(objectiveTargetSize) - np.sqrt(area)
    
    # Constantes para ajustar sensibilidad (cómo de lineal es la sigmoidal)
    kp_position = 1  # Factor proporcional para posición
    kp_size = 1      # Factor proporcional para tamaño
    
    # Función sigmoid para suavizar la respuesta 
    # k regula cómo de lineal es la sigmoid
    def sigmoid(x, k=1):
        return 1 / (1 + np.exp(-k * x))
    
    # Ajuste de velocidad lineal (v) basado en error de tamaño
    # Si la bola es más pequeña que el objetivo -> avanzar
    # Si la bola es más grande que el objetivo -> retroceder
    v_adjusted = kp_size * (v * sigmoid(error_size) - (v/2))
    
    # Ajuste de velocidad angular (w) basado en error de posición
    # Si la bola está a la derecha del objetivo -> girar izquierda
    # Si la bola está a la izquierda del objetivo -> girar derecha
    w_adjusted = kp_position * (w * sigmoid(error_x) - (w/2))
    
    print("v_adjusted: ", v_adjusted)
    print("w_adjusted: ", w_adjusted)
    
    return v_adjusted, w_adjusted


# Devuelve la velocidad (v,w) necesaria para dar vueltas sobre sí mismo 
# hasta que encuentra una bola 
def calcSearchSpeed(w):
    # hace elipses por si la bola está lejos
    v = 0.2
    return v, w
