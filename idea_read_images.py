import cv2
import numpy as np
import picamera
import picamera.array

# Inicializar la cámara
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30

# Capturar imagen en memoria
with picamera.array.PiRGBArray(camera) as output:
    camera.capture(output, format="bgr")  # Captura en formato OpenCV (BGR)
    image = output.array  # Obtener la imagen en memoria

# Invertir la imagen (flip horizontal y vertical)
image = cv2.flip(image, -1)

# Convertir a escala de grises
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Aplicar desenfoque para reducir ruido
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# Aplicar detección de bordes con Canny
edges = cv2.Canny(blurred, 50, 150)

# Aplicar erosión para eliminar ruido pequeño
kernel = np.ones((5, 5), np.uint8)
eroded = cv2.erode(edges, kernel, iterations=1)

# Mostrar las imágenes
cv2.imshow('Original', image)
cv2.imshow('Gris', gray)
cv2.imshow('Bordes', edges)
cv2.imshow('Erosion', eroded)

cv2.waitKey(0)
cv2.destroyAllWindows()

# Cerrar la cámara
camera.close()
