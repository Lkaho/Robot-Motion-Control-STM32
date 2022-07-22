# Robot-Motion-Control-STM32

This is my undergraduate graduation project. The main content of the project is to modify an electric car into a driverless car with four-wheel steering.

<img src="/home/kaho/Workspaces/ChassisControl/images/original.png" alt="original" style="zoom:75%;" />

## 1. Main Contribution

- Responsible for the modification of the unmanned chassis, mainly including the 3D modeling and assembly of the rear wheel steering system and braking system.
- Design the software part of the motion control controller based on STM32.

## 2. Chassis Modification Part

- Rear wheel steering system

  - before modification

    <img src="/home/kaho/Workspaces/ChassisControl/images/before_mod.jpeg" alt="before_mod" style="zoom: 67%;" />

  - after modification

    - 3D modeling

      ![rear_steer_system](/home/kaho/Workspaces/ChassisControl/images/rear_steer_system.png)

    - Assembling

      ![after_mod2](/home/kaho/Workspaces/ChassisControl/images/after_mod2.jpeg)

- Brake system

​		Using the cable-pulling scheme and adopting the step motor as the power source to modify the braking system.

​								![brake system](/home/kaho/Workspaces/ChassisControl/images/brake system.png)		

## 3. STM32 Controller

- Hardware system architecture

  ![Hardware System](/home/kaho/Workspaces/ChassisControl/images/Hardware System.png)

- Main Module

  - Serial communication : Communicate with on-board computer(see Hardware/STMROS)

  - DAC: Control driving system (see Hardware/DAC)
  - CAN: Control the steering system and  collect chassis state(see Hardware/CAN)
  - Stepdriver: Control brake system(see Hardware/STEPDRIVER) 
  - RS485: Communicate with the driving motor controller and collect vehicle speed via modbus protocol. (see Hardware/RS485)

## 4. Test

![motion_control_test](/home/kaho/Workspaces/ChassisControl/images/motion_control_test.gif)
