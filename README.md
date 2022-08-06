# Robot-Motion-Control-STM32

This is my undergraduate graduation project. The main content of the project is to modify an electric car into a driverless car with four-wheel steering.

![img](https://github.com/Lkaho/Robot-Motion-Control-STM32/blob/main/images/original.png)

## 1. Main Contribution

- Responsible for the modification of the unmanned chassis, mainly including the 3D modeling and assembly of the rear wheel steering system and braking system.
- Design the software part of the motion control controller based on STM32.

## 2. Chassis Modification Part

- Rear wheel steering system

  - before modification

    ![img](https://github.com/Lkaho/Robot-Motion-Control-STM32/blob/main/images/before_mod.jpeg)

  - after modification

    - 3D modeling

      ![img](https://github.com/Lkaho/Robot-Motion-Control-STM32/blob/main/images/rear_steer_system.png)

    - Assembling

      ![img](https://github.com/Lkaho/Robot-Motion-Control-STM32/blob/main/images/after_mod2.jpeg)

- Brake system

​		Using the cable-pulling scheme and adopting the step motor as the power source to modify the braking system.

​								![img](https://github.com/Lkaho/Robot-Motion-Control-STM32/blob/main/images/brake%20system.png)	

## 3. STM32 Controller

- Hardware system architecture

  ![img](https://github.com/Lkaho/Robot-Motion-Control-STM32/blob/main/images/Hardware%20System.png)

- Main Module

  - Serial communication : Communicate with on-board computer(see Hardware/STMROS)
  - DAC: Control driving system (see Hardware/DAC)
  - CAN: Control the steering system and  collect chassis state(see Hardware/CAN)
  - Stepdriver: Control brake system(see Hardware/STEPDRIVER) 
  - RS485: Communicate with the driving motor controller and collect vehicle speed via modbus protocol. (see Hardware/RS485)

- Controller Function Test(Simulation)

  ![controller_sim](/home/kaho/Workspaces/ChassisControl/images/controller_sim.jpeg)

## 4. Test

![img](https://github.com/Lkaho/Robot-Motion-Control-STM32/blob/main/images/motion_control_test.gif)
