# STM32_NeuralNet_MovementDetection
Innovative project over neural networks and embedded systems. Check the [report](Rapport_PX504_BarriereCanetLeDongeMoriceau.docx.pdf) if you want to get extra informations and more precision about the results of this project. 

# Overview
This project aims at creating a Neural Network for STM32(L476RG) to do motion recognition with data from accelerometer. The neural net used is a multilayer perceptron and it has (currently) 3 layers. 
The goals are :
- To use a neural network model (testing) on an stm32 to perform motion recognition
- To train the neural network on the stm32 itself
- To perform the testing algorithm on an FPGA to increase speed
- To compare performances between the stm32 alone and the stm32 with the fpga
- [Check the report](Rapport_PX504_BarriereCanetLeDongeMoriceau.docx.pdf)

# Requirements
* STM32CubeMX : to edit project configuration (clock, gpios...)
* SystemWorkbench for STM32
* Accelerometer (ADXL345 used in this project)
* STM32 : STM32L476RG (Nucleo) used here

# Credits
Four motivated Esisar students !
* Louka BARRIERE
* Gauthier CANET
* Romain LE DONGE
* Alexandre MORICEAU

