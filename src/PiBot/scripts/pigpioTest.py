from gpiozero import Motor
from time import sleep

motor1 = Motor("25", "18")
motor2 = Motor(14, 15)
motor1.forward(0.5)
motor2.forward(0.5)
sleep(1)
