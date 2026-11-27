# STM32F303 LED Dance

Press the user button on the STM32F303 Discovery board to cycle through four patterns on its eight onboard LEDs.

<p align="center">
  <img src="Assets/demo.gif" alt="Four LED patterns on the STM32F303 Discovery board" width="460">
</p>

## How it works

The user button on `PA0` triggers the `EXTI0` interrupt. The HAL callback advances the pattern index. The main loop reads that index and runs the selected routine on the LEDs connected to GPIOE.

The patterns and callback are in [main.c](Core/Src/main.c). The [CubeMX configuration](01LED_Mode_Toggle.ioc) shows the GPIO and interrupt setup.

## Flash the demo

1. Import the project into STM32CubeIDE.
2. Connect the board over ST-LINK and build the Debug configuration.
3. Flash the firmware, resume execution and press the user button to switch patterns.
