## Power Supplies

### Jetson
To power the Jetson we need a power consumption ranging between 7.5 to 15 W depending on the workload. The input will usually be a 5V-4A (20W) power supply.


### Motors
12V or 24V LiPo (Lithium Polymer Batteries) with high mAh capacity to ensure longer operational time. 


A good idea will be to seperate the Jetson and other electronic components (Like Raspberry Pi Pico) from the motors. We do not want voltage drops and electrical noise. Do this with seperate power supply or voltage regulator.

### Conclusion on what we need:
- 4S 14.8V Lipo Battery (5000mAh)
- seperate 5V 4A regulator from LiPo to Supply Jetson.
- Power motors directly from 12 or 24V battery, or through motor controllers that regulate voltage.



## Switches
For safety reasons and convenience we will have switches between power supplies and components. With these switches we can control what parts of the robot that are powered at a specific time.
After research best option will probably be toggle or rocker switches with appropriate current and voltage (12/24V DC for motors)

One switch will control the entire robot, cutting off power to all the components. Another one will independently control the Jetson, one will disconnect the motor system so that we can power the robot but not activate the motors. The last one will power the Raspberry Pi Pico independently (optional but could be convienient for debugging).

### Conclusion on what we need:
- Master Switch: Use a heavy-duty toggle or rocker switch rated for the voltage of your battery (e.g., 12V or 24V) and at least 10A current.
- Jetson Switch: A smaller switch rated for 5V DC and 5A.
- Motor Switch: A high-current-rated switch, such as a 12V 20A rocker switch.
- Raspberry Pi Pico Switch: Similar to the Jetson, you can use a smaller switch rated for 5V DC and 2-5A.
