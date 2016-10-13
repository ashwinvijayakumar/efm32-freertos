# efm32-freertos
FreeRTOS demo project for EFM32 based boards

# How to build the project
1. Install SiLabs Simplicity Studio by following instructions on https://www.silabs.com/products/mcu/Pages/efm32-simplicity-studio-getting-started.aspx
2. Run Simplicity Studio and create a workspace. Say v4_workspace
3. Download https://sourceforge.net/projects/freertos/files/FreeRTOS/V9.0.0/FreeRTOSv9.0.0.zip/download and extract it to the top level directory of your workspace (v4_workspace)
4. Now git clone this project into to the top level directory of your workspace (v4_workspace)
5. Import this project into Simplicity workspace as defined in "Building and executing the demo application" section of http://www.freertos.org/EFM32-Giant-Gecko-Pearl-Gecko-tickless-RTOS-demo.html
6. Open a serial terminal (ex. PuTTY) with 8-n-1 115200 BAUD settings
6. Build and flash the demo using the built-in flashing tool
7. You should see the following on the terminal:
EFM32 FreeRTOS demo!
Temp = 24396 | Humidity = 54592 | Button = 0



