# DJITelloPy_E205
Harvey Mudd College // Spring 2020 // E205 Systems Simulation // Final Project.  Developed by Max Maleno '20 and Kevin Shoyer '20.

Repository is originally forked from damiafuentes' DJITelloPy due to its robustness and strong foundations for communicating with the Tello using Python 3.  It's an awesome project!

Signicant Change log:

2020-04-07 -- datalogging (both Tello telemetry and videostream) is implemented.  CSV file and AVI video is generated for every session.

## Scripts and functionallity
posit_proccess.m
    functionallity: was used to experiment with the control outputs of the drone. 
    dependencies: files in "callibration data logs" folder. You will need to fix the path to these files. Look in post_proccess to find description of test.
CameraCallibrationMatlab
    functionallity: This folder contains the matlab camera callibration toolkit files
    dependencies: none

## Walkthrough/tutorial
A nice writeup will be written in the near future...

### Basic movement in keyboard_control.py

The keyboard controls for keyboard_control.py are:
- T: Takeoff
- L: Land
- Arrow keys: Forward, backward, left and right.
- A and D: Counter clockwise and clockwise rotations
- W and S: Up and down.

### How to use atm:

1.  Go into Apriltag/apriltag-master/python
2.  run "python3 ../../../keyboard_control.py"
3.  After performing trial, run following command: "python3 apriltag_vid_generator.py -c -k '(865.4575, 867.2598, 481.5149, 361.2172)' -s .16 poop"
3.0 Use this command for Max's drone: python3 apriltag_vid_generator.py -c -k '(912.906618726668, 935.153100619758, 470.318057440520, 367.001678120807)' -s .16 poop
3.1 you need to say "poop" at end of line because it is expecting one more argument.  TODO change this in future
3.2 those are camera params kevin gave me for Tello

### Notes
- to be replaced

## Resources
- [E205 SP20 website](https://sites.google.com/g.hmc.edu/e205)
- [Tello product website](https://www.ryzerobotics.com/tello)
- [Tello SDK documentation](https://dl-cdn.ryzerobotics.com/downloads/tello/20180910/Tello%20SDK%20Documentation%20EN_1.3.pdf) 

## Original Author

* **Damià Fuentes Escoté** [DJITelloPy](https://github.com/damiafuentes/DJITelloPy)

