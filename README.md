# Potentiostat
A potentiostat is an electronic device that measures and controls the voltage difference between two electrodes in an electrochemical cell.

This project uses an STM32F072 microcontroller to create a potentiostat. The STM32 receives control parameters (voltage and time) from a laptop GUI over UART, applies the voltage to an electrode via its DAC, measures the voltage difference between two other electrodes, and sends the data back to the laptop. The laptop then plots the voltage vs. time graph in real-time.

Features
1. Control Voltage: Set voltage to be applied to the working electrode.
2. Measure Difference: Continuously measure voltage difference between two electrodes.
3. Real-time Graph: Laptop GUI plots voltage vs. time.
4. UART Communication: STM32 and laptop communicate over UART.

Hardware Requirements
1. STM32F072 board (e.g., Nucleo-F072RB)
2. Electrochemical cell with three electrodes (working, reference, counter)
3. USB-to-UART cable (if not using onboard USB)
4. Power supply for STM32 and potentiostat setup
5. Laptop with GUI software

Software Requirements
1. STM32 Firmware: Developed using STM32CubeIDE and CubeMX
2. Laptop GUI: Python script to control the potentiostat and plot data

Setup
1. STM32 Firmware
Use STM32CubeMX to configure the DAC, ADC, and UART peripherals.
Compile and flash the firmware using STM32CubeIDE.
2. Laptop GUI
Install Python dependencies:
python gui/potentiostat_gui.py
Set the voltage and time in the GUI and start the experiment. The data will be plotted in real-time.
