# Robot Chicken Mainboard

Programmed with PlatformIO for an [STM32H743IIT6](https://www.amazon.com/dp/B0F28RDHB6)
This project is built to locally run my robot chicken's walking network (along with other networks in the future for standing, getting up, etc)
Uses the output of onnx2c to run the model, then uses CAN to communicate with the other boards. Also uses I2C to communicate to two screens (for eyes) and to an MPU6050 for gyro and accel data for the network.
