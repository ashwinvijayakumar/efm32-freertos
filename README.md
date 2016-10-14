# efm32-freertos
FreeRTOS demo project for EFM32 based boards

# How to build the project
1. Install SiLabs Simplicity Studio by following instructions on https://www.silabs.com/products/mcu/Pages/efm32-simplicity-studio-getting-started.aspx
2. Run Simplicity Studio and create a workspace. Say v4_workspace
3. Download https://sourceforge.net/projects/freertos/files/FreeRTOS/V9.0.0/FreeRTOSv9.0.0.zip and extract it to the top level directory of your workspace (v4_workspace)
4. Now git clone this project (ashwinvijayakumar/efm32-freertos) into the top level directory of your workspace (v4_workspace)
5. Import this project into Simplicity workspace
  - In Simplicity Studio: File > Import... > General > Existing Projects into Workspace > Next > Browse > v4_worspace/efm32-freertos > Finish
  - The project has relative/symbolic links to Simplicity SDK and FreeRTOSv9.0.0, so make sure the Simplicity is installed in it's default location (C:\SiliconLabs\SimplicityStudio\v4\developer\sdks\exx32\v4.4.1), and FreeRTOSv9.0.0 is at the same directory level as the project. i.e. v4_workspace 
6. Open a serial terminal (ex. PuTTY) with 8-n-1 115200 BAUD settings
6. Build and flash the demo using the built-in flashing tool
7. You should see the following on the terminal:
EFM32 FreeRTOS demo!
Temp = 24396 | Humidity = 54592 | Button = 0
8. Hit 1 or 0 on the serial terminal to switch the on-board LED ON and OFF



