# Team 4653 Irrational DoorKeepers Team Code

This is the team code library for the FTC team #4653 Irrational DoorKeepers from Northern Guilford High School in Greensboro, NC.
This repository contains all of the coding we did throughout the 2020-2021 FIRST Ultimate Goal Season, including all Autonomous, TeleOp, and Calibration Testing.
And....MAY THE FORCE BE WITH YOU!!!

## Change Log:

- 9/14/2020 - Added pixel count and pixel density support for the VuforiaDetector and the FTC_IDK_VISION_LIB support in the TeamCode.

- 9/28/2020 - Added RGB 565 encoding support to the FTC_IDK_VISION_LIB, setup TeleOp and Autonomous for the Ultimate Goal Season. 
Changed the image in AutonomousMapping application for Ultimate Goal.

- 9/29/2020 - Updated and finished preliminary autonomous for parking and main autonomous. Fixed major distance bug in the AutonomousMapping application.

- 10/4/2020 - Fixed a bug regarding the Positional Movement and the vision library, also was able to run the parking autonomous successfully. 
Also found another bug in the vision library with Vuforia that in the process of fixing.

- 10/6/2020 - Added Machine Learning to the FTC_IDK_VISION_LIB, and fixed the vision library for Ultimate Goal and overall. 
Also, restructured the code and made the code more readable. 
Additionally, restructured project and added TensorFlow and OpenCV vision detections as backups.

- 10/7/2020 - Added Coordinate Mapping and YOLO ML Detection to Vision Lib. Also, added EasyOpenCV Support and DogeForia Support along with a start on the 
Vision Positional Mapping Software. 

- 10/10/2020 - Changed the Positional Movement Software (PMS) to be single movements at a time for more customization. Also, fine tuned the values and vision inputs to make
the main autonomous work on the blue and red allicances. We plan to work on the Vision Positioning Mapping Software (VPMS) further.

- 10/11/2020 - Changed Initlialization method for the FTC_IDK_VISION_LIB and started to implement Vision Position Tracking, Automation, and Detection Visualization with the 
Vision Library. Changed the Joystick controls for the arm and the chassis to make things more efficient and stable while driving.

- 10/13/2020 - Added new Gyro Stabilization feature, tweaked Autonomous Programs, Started to add Vision Position Mapping and PID Controller. Setup new Joystick controls for
overall systems including drive train and arm, build team is adding more mechanisms, so have to keep up. Tweaked Vision profiling, now Vision works under many different lighting
conditions and systems.

- 10/16/2020 - Added new Motion Profiling Controller and started on motion profiled turning algortihm and vision positioning work. Need to integrate further and
test changes in autonomous program. Also, will add further PID controls if needed for the motion profiling.