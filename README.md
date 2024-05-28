# Sistema de Control (Flight Controller) para avión RC con Arduino Nano

![nanofc](https://github.com/gnomada/TFS-NANO-FC/blob/main/assets/photo1.jpg?raw=true)

![motor](https://github.com/gnomada/TFS-NANO-FC/raw/main/assets/video1.mp4)


Este proyecto es un sistema de control para aviones RC basado en Arduino, utilizando el controlador de servos PWM Adafruit PCA9685 y motores servo. El sistema lee la entrada de un transmisor RC y ajusta el acelerador, el timón (yaw), la profundidad (pitch) y el balanceo (roll) del avión en consecuencia.

He utilizado un mando y receptor Fly Sky FS T4-B y he seguido los ejemplos/código de [Arduproject](https://arduproject.es/conceptos-generales-sobre-drones/).

El variador es uno genérico de AliExpress.

He usado la placa PCA9685 de Adafruit para que el mismo hardware sea compatible con más servos en el futuro y no tener que hacer muchos cambios de código, pero puedes prescindir de esto.


## Componentes
- Arduino Nano v3
- Controlador de Servos PWM Adafruit PCA9685
- Motores Servo
- Controlador Electrónico de Velocidad (ESC)
- Transmisor y Receptor RC
- Batería

## Definiciones de Pines
- `pin_INT_Throttle`: Pin 11 (Acelerador - Canal 3 del RC)
- `pin_INT_Yaw`: Pin 10 (Timón - Canal 4 del RC)
- `pin_INT_Pitch`: Pin 12 (Profundidad - Canal 2 del RC)
- `pin_INT_Roll`: Pin 9 (Balanceo - Canal 1 del RC)
- `status_led`: Pin 13 (LED de Estado)
- `pin_esc`: Pin 5 (PWM del ESC)

## Bibliotecas
- `Wire.h`
- `SPI.h`
- `Adafruit_PWMServoDriver.h`
- `Servo.h`
- `EnableInterrupt.h`


## Funciones
### Control de Servos
- `setServo(uint8_t n_servo, int angulo)`: Establece la posición de un servo.
- `mapYawToServoPosition(int yawValue)`: Mapea la entrada del timón a la posición del servo.
- `mapPitchToServoPosition(int pitchValue)`: Mapea la entrada de profundidad a la posición del servo.
- `mapRollToServoPosition(int rollValue)`: Mapea la entrada de balanceo a la posición del servo.
- `mapThrottleToServoPosition(int throttleValue)`: Mapea la entrada del acelerador a la señal del ESC.
- `setThrottle(int throttleValue)`: Establece el valor del acelerador del ESC.

### Manejadores de Interrupciones
- `INT_Throttle()`: Maneja la entrada del acelerador.
- `INT_Pitch()`: Maneja la entrada de profundidad.
- `INT_Roll()`: Maneja la entrada de balanceo.
- `INT_Yaw()`: Maneja la entrada del timón.

## Bucle Principal
1. Espera 10 ms antes de cada iteración del bucle.
2. Actualiza las posiciones de los servos basándose en los valores de entrada del RC.
3. Establece el acelerador utilizando el ESC.
