# Climbing Wheelchair

This repo focus on the program  in the embedded system(STM32F429VI) on the wheelchair.
The objective of the project is to allow the wheelchair user climbing up and down a curb safely and smoothly.

## Requirement
- STM32CubeIDE > 1.7.0
  All OSes: [click here for installation instructions](https://www.st.com/en/development-tools/stm32cubeide.html)


## Installation
STM32CubeIDE is used to generated the overall code and structure.
In Project Manager/Code Generator, select "Generate peripheral initialization ..."

Tips: Add the code in between the section 
- `/* USER CODE BEGIN XX */` and 
- `/* USER CODE END XX */`

Therefore, if the code regenerated through CubeMX, the code added will be saved.

## Hardware and Peripheral Used
| Hardware | Peripheral Used | File related |
| --- | ----------- | ----------- |
| Joystick (AD7606) | SPI1 | spi.c / adc.c / wheelchair.c /  main.c |
| Climbing motor (BD25L) | Rear: TIM8 CH4  Back: TIM1 CH2 | tim.c / bd25l.c |
| Climbing motor encoder | CAN1 / DMA1 | can.c / dma.c / encoder.c / main.c |
| MPU6050 | I2C1 | i2c.c / mpu6050.c |
| Hub Motor (X2_6010S) | UART3 | uart.c / X2_6010S.c / main.c |

*\*main.c indicate there is callback function in it*






