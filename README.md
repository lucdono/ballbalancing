# ballbalancing
The project is a simple control system able to keep a ball at a given set point.
It is developed using an NXP Freedom K64 board (Cortex-M4).
It works using a resistive touch screen that provides the location of the ball on the plate. The location is fed to a pair of PID controllers. The PID controllers along with two Kalman filters, are responsible of generating the appropriate signal for the servos, in order to drive the ball in the desired location.
Three modes are available via a push button:  
 - center: keep the ball in the center (green blinky led)
 - circle: move the ball along a circular trajectory (blue blinky led) 
 - vertex: move the ball to the vertex of a square (red blinky led)

If no touch is received (i.e. no ball on the plate), the plate is set to it's default position (0 degrees in both X and Y) and the led is off.
