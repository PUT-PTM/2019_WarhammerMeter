# 2019_WarhammerMeter

## Description
The objective of this project is to creata a tool that will allow the user to find the distance between tw spots the device is pointed to.

## How it works?
The core concept is simple:
- using two separate rangefinders, (with LED lasers for easier targeting), find the distance between the two measured objects and the device.
- using a potentiometer define the angle of both range finders.
- use the angle and the measured distances to calculate the third side of the triangle - the actual distance between the two objectes
- display the result on a LCD display

## Used Protocol
- I2C for LCD
- UART for the rangefinders 

## Hardware
For this project the STM32 NUCLEO-F072RB - STM32F072RB ARM Cortex M0 was used

Autorzy projektu:
Marcin Trocha
Michał Labuda
