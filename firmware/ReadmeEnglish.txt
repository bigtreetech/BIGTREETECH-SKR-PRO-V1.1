1. Introduction to firmware version naming rules in BIGTREETECH-SKR-PRO-V1.1:
	
For example: stm32f407zgMarlin-bugfix-2.0.xV20190627-12864-5160SPI	

stm32f407zg 		The model of the main control chip
Marlin-bugfix-2.0.x                   Marlin firmware version
V20190627		Marlin 2.0 firmware version downloaded on June 27, 2019
12864			Screen configured in firmware
5160			Model of motor driven chip in firmware.
SPI 			Driver chip SPI driver mode
usually			General-purpose,16-segment by default
uart			Driver chip uart driver mode
16div 			16 Subdivisions


Note: if you want to design your own version naming rules, folder names must not exceed 64 characters in length, 
marlin 2.0 project files in other folders, folder nesting depth can not exceed 3, 
otherwise, when opening the project compilation project, there will be unexpected compilation errors!