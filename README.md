Github containing the code for the Robotics subject in the Computer Engineering Degree of Universidad de Zaragoza.

Autores: 839385 Miguel Arasanz, 844417 Andrei Dumbrava, 841723 Diego Roldán.

**Uso:**

Genera las claves ssh ejecutando key-gen.sh

Para ejecutar el fichero Python en cuestión, configurar en exec.sh y ejecutarlo.

**Cambios realizados sobre la plantilla proporcionada:**

- La función `robot.TrackObject` recibe distintos parámetros:

```python
def trackObject(self, v_base=0.4, w_base=np.pi/2, catch=True, targetX=160, minObjectiveTargetSize=4500, maxObjectiveTargetSize=8500, detection_tolerance=30, maxYValue=32, colorMasks=None):
        """
        Tracks the ball and tries to catch it. The robot will move towards the ball until it is in the objective area
        and then it will try to catch it. If the catch is successful, the robot will stop. If the catch is not
        successful the robot will start looking for the ball again.
        :param v_base: base linear speed
        :param w_base: base angular speed
        :param catch: boolean to indicate if the robot should catch the ball
        :param targetX: x position of the target
        :param minObjectiveTargetSize: minimum area of the ball to consider it as the target
        :param maxObjectiveTargetSize: maximum area of the ball to consider it as the target
        :param detection_tolerance: tolerance in the x position of the ball
        :param maxYValue: maximum y position of the ball
        :param colorMasks: list of color masks to detect the ball
        """
```
