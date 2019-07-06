1. Sometimes an error occurs when compiling this branch for the first time. Please Close vscode and reopen, then compiling will be normal.
2. if you want use tmc2208 uart mode, please copy [3 files](https://github.com/bigtreetech/BIGTREETECH-SKR-PRO-V1.1/tree/master/firmware/SoftwareSerial) in SoftwareSerial folder to your platformio install path for example "C:\Users\<your user name>\.platformio\packages\framework-arduinoststm32\cores\arduino" and comment out the "typedef int8_t pin_t" in file "Marlin\src\HAL\HAL_STM32\HAL.h" [line 135](https://github.com/bigtreetech/BIGTREETECH-SKR-PRO-V1.1/blob/dd073164e9b15835fc3b09e865ce44cd31d26688/firmware/Marlin-SKR-Pro/Marlin/src/HAL/HAL_STM32/HAL.h#L135)

Note: if you want to design your own version naming rules, folder names must not exceed 64 characters in length, 
marlin 2.0 project files in other folders, folder nesting depth can not exceed 3, 
otherwise, when opening the project compilation project, there will be unexpected compilation errors!
It is recommended that the file name should not be named too long, the shorter the better. 
File nesting depth should not be too much, the less nesting, the better.
