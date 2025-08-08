# Soft Neck Robot
This repository contains my Master Thesis project, in which I had to control the bending of a **Soft Neck Articulation** through the feedback of a **Soft TPU/CB Flexible Sensor**.

The research has been held at the **Robotics Lab** at **Universidad Carlos III de Madrid**: <https://softroboticslab.gitlab.io/website/index.html>

<img width="1041" height="470" alt="neck3D" src="https://github.com/user-attachments/assets/5dbd9328-31aa-4bd5-a425-609f44e81967" />



*The Soft Sensor requires the multimeter or the Arduino in order to be sensed, according to the chosen code.*

## Multimeter
### Multimeter Setup
1. To connect the multimeter, check the presence of the code `multimeter_communicationLinux.cpp` in the *Soft_neck_Soft_sensor/main* folder: this code is compatible with every OS since it is based on the SerialComm.h library.
2. Press Power On
3. Switch to "Four-Wires Mode": a) Press Shift and then b) Press 4W
   
<img width="1280" height="616" alt="multimeterHP" src="https://github.com/user-attachments/assets/3a3e9f02-f525-4f30-9102-3cf902a60701" />



### Multimeter Common Issues
1. #### ERROR displayed during multimeter initialization
    The code automatically set the multimeter to LOCAL mode and then to REMOTE mode. When the code execution crashes or is forced to quit, this action may generate an error during the following test. So after an interrupted test, shut down and restart the multimeter before starting a new one.\
    <img width="436" height="186" alt="multimeter_error" src="https://github.com/user-attachments/assets/7c53306f-2be9-43e4-a0fa-b284bca757be" />



## Arduino
### Arduino Setup
1. Open the Arduino IDE
2. Modify `Filter.ino` in *Soft_Neck_Soft_Sensor/Filter* folder with the correct pinout, baud rate, filter parameters, resistance value (of the Tension Divider Circuit).
3. Upload `Filter.ino` on the Arduino

### Arduino Common Issues
1. #### Arduino not detected
    - Arduino IDE should be opened before running any code on Qt Creator: this helps initializing the connection between PC and Arduino. 
    - Check that the Serial Monitor and Serial Plotter of the Arduino IDE are closed: otherwise the data is sent to them instead of Qt Creator.
2. #### Wrong resistance output
    Check that the fixed resistance (of the Tension Divider Circuit) in `Filter.ino` is the same used in the breadbord and check that the cable connections of the breadbord are correct.




## General
### Setup and Recommendations
1. Neck's tendons need to be manually tightened before any demo without the neck bending in any direction and avoiding to compress the soft link (for a perfect vertical position help yourself with a bubble level).
Do it with the motor being shut down and be sure to tighten clockwise direction: the tendon should be on the outside.

    <img width="2028" height="831" alt="neck_tendons" src="https://github.com/user-attachments/assets/173b440c-cd5a-4f02-8b12-bc06d4731f94" />



2. Check that every cable is connected: power, CAN bus, IMU and Arduino/multimeter.
3. Check in the chosen code, for IMU and Arduino/Multimeter, that the specified Serial Ports are the same of the actual used ones. For IMU and multimeter you check it from the Terminal with the command:
   ```bash
   dmes | grep ttyUSB
   ```
   For Arduino you check it with the command:
   ```bash
   dmes | grep ttyACM
   ```
   
4. When running the code, be ready to shut down the motors in case of any control issue during the tests to avoid breaking the soft link.


### Running the Code
1. Build with `CMakeLists.txt` in the outer *Soft_Neck_Soft_Sensor* folder (it will call all the `CMakeLists.txt` of the sub-folders automatically building every source file).
2. Choose your code in *Soft_Neck_Soft_Sensor/main* folder and open it through Qt Creator:


    |Name|Multimeter/Arduino|Goal|Feedback|
    |----|------------------|----|--------|
    | `9.cpp`  | Multimeter | Control | IMU |
    | `10.cpp` | Multimeter | Sys Identification | Open Loop |
    | `14.cpp` | Arduino | Control | FlexSensor |
    | `15.cpp` | Arduino | Sys Identification | Open Loop |


### Generating New Reference Signals
In each code, after IMU calibration commands, there is a section dedicated to the generation of Reference Signal introduced and followed by a line of ****.

In this section, reference vectors can be created by:
1. generating simpler child vectors as *"target0", "target1"* with ad hoc methods, being able to specify: amplitude, rate, duration, number of cycles, frequency...
2. concatenating child vectors to a parent vector like *"target_pitch"* (target_pitch = target0, target1, target2, target3...)


```bash
std::vector<double> target_pitch;

std::vector<double> target0;
referenceCyclicRamp(target0, dts, 0.35, 0.17, 3);
target_pitch.insert(target_pitch.end(), target0.begin(), target0.end());

std::vector<double> target1;
referenceStep(target1, dts, 0.52, 20.0);
target_pitch.insert(target_pitch.end(), target1.begin(), target1.end());
```


**ATTENTION:** code `14.cpp` also contain a preload_pitch refence signal that is tracked in position OPEN-LOOP before target_pitch (preload_pitch = preload0, preload1, preload2...). **DO NOT COMMENT** it unless you are sure that your flex sensor does not present any initial drift. If the drift persist for more time, you can increase the duration of the preload motion.


### Saved Data
- The data are automatically saved as .csv files in the *Soft_Neck_Soft_Sensor/CSVdata* folder after each test. The folder is not emptied before the saving, so be careful to not use data saved from previous tests.
- If you want to save more variables, check if you want to save a vector or a 2D vector: choose between the methods exportVectorToCSV() and export2DVectorToCSV().


### Common Issues
1. #### Executions stops before IMU calibration
    The IMU could not close correctly every time, so the execution of the program may stop, usually printing this last line:
   ```bash
   Port /dev/ttyUSB0 has been correctly initialized
   ```
   To solve it, just Force Quit and RUN again.

2. #### Loose tendons
    After many tests or with a new neck, tendons can get loose and tangle (unknown reason, motor wear? asymmetric system?), leading to position drift and saturation.
    To solve it you can go to `SoftCC3Tendon.cpp` file in *Soft_Neck_Soft_Sensor/lib/robot-device/robots* containing the robot kinematics, and change the last lines of the SetRollPitchArcLen() method:
   ```bash
   if(tendons[0]<0) tendons[0] = 1.1*tendons[0];
   if(tendons[1]<0) tendons[1] = 1.1*tendons[1];
   if(tendons[2]<0) tendons[2] = 1.1*tendons[2];
   ```
   Since the tendons get loose only during negative velocities, we want to increase their absolute value only when negative (1.1 = increased by 10%, 1.3 = increased by 30% ...).
   Play with this amplification value until you have no more drift.

   For new necks, set the correct soft link height *arc_length*.
