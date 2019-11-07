# STM32CubeIde-NAZE32

Example code for programming the Naze32 board in STM32CubeIDE

Currently implemented:
* I2C (Using existing library)
* PWM
* UART (DMA)
* PID function
* Complimentary filter function
* Reading from MPU6050 (through I2C)
* Reading from SR04 Ultrasonic Proximity Sensor

**BEWARE:** Compiling with versions of CubeIDE newer than 1.0.2 currently is not working, due to changes in how directories are managed in newer versions.
