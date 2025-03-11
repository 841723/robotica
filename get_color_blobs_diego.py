import cv2
import numpy as np
import os


def detect_ball(img_BGR, show:bool=False):
    # # Leer la imagen
    # img_BGR = cv2.imread(img)

    # print("Procesando imagen:" + img_BGR.apply(str))

    # Convertir a HSV
    img_HSV = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2HSV)

    # Definir los rangos de color rojo en HSV
    redMin1 = np.array([0, 70, 50])      # Rango bajo
    redMax1 = np.array([10, 255, 255])

    redMin2 = np.array([170, 70, 50])    # Rango alto
    redMax2 = np.array([180, 255, 255])

    # Crear máscaras y combinarlas
    mask1 = cv2.inRange(img_HSV, redMin1, redMax1)
    mask2 = cv2.inRange(img_HSV, redMin2, redMax2)
    mask_red = cv2.bitwise_or(mask1, mask2)

    # Encontrar contornos en la máscara
    _, contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx, cy, area = 0, 0, 0 

    # Si se detectan contornos
    if contours:
        # Tomar el contorno más grande (suponiendo que es la pelota)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Calcular el centroide (posición de la pelota)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = 0, 0  # Evitar división por cero

        # print(f"Pelota detectada en posición: ({cx}, {cy}) con área: {area}")

        if show:
            # Dibujar solo el punto en el centro
            cv2.circle(img_BGR, (cx, cy), 3, (0, 255, 0), -1)  # Verde, relleno

    if show:
        # Mostrar la imagen con el punto central marcado
        cv2.imshow("Pelota Detectada", np.stack([mask_red]*3, axis=-1)) 
        cv2.imshow("Pelota Detectada con Centroide", img_BGR)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    return cx, cy, area



def detect_ball_all_files(path:str):
    # Loop over the files in the directory
    for filename in os.listdir(path):
        if filename.endswith(".jpg"):
            # Detect the ball in the image
            detect_ball(path+"/"+filename, show=True)


if __name__ == "__main__":
    detect_ball_all_files("fotos/imagenes")