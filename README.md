# Team 4653 Irrational DoorKeepers Team Code

This is the Code library for FTC Team #4653 Irrational DoorKeepers from Northern Guilford High School in Greensboro, NC.
This Repository Contains All of the Coding We Did, Including All Autonomous, TeleOp, and Calibration Testing. And....MAY THE FORCE BE WITH YOU!!!

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

- 10/21/2020 - Added further Motion Profiling Control and automation of the Arm and claw. Further improvements to the Vision Positioning System and testing of the 
motion profiling and automation to come. Fixed bugs in Motion Profiling and changes to autonomous, more Motion Profiling and Automation Changes to come.

- 10/25/2020 - Fixed Autonomous Bugs and Added Support for New Wobble Goal Arm. Also Tested Arm and Chassis Movements. Also, Fixed Motion Profiling and Move-to-Position.
Wil Continue to Work on Vision Positioning and Turning Algortihms.

- 11/5/2020 - Added an Another Powershot to Autonmous, Added Vision Positioning Support, Coded for Intake and Flywheel/Shooter, Fixed and Tuned Motion Profiling, 
Added a Custom GamePad Class, Tuned and Automated the TeleOp Further, and Made Code Easier and More Modular to Work With.

- 11/18/2020 - Began Tweaking Autonomous and Mechanisms Parameters to Fine Tun Robot Movements for Perfection. We Are Still Working on Developing Shooter and More Consistent 
Shooting During Autonomous Especially.

- 11/21/2020 - Continued upon Improving the Autonomous Programs (Main and Side) for Upcoming Scrimmage in NC. Also, We Fixed and Updated the Gyro-Correction Algortihm.
Lastly, We Continued Working on the FTC_IDK_VISION_LIB By Adding Noise Correction on Images.

- 12/1/2020 - Added Dynamic Autonomous Direction Changes, Better Path Planning Implementation, Better Turning to Position, and Continued Work on Vision Lens Correction. 

- 12/4/2020 - Changed Automation Mechanisms to Better Program and Interface with. Tweaked Autonomous and Scoring Strategy to Focus More on High Goal Consistency.

- 12/6/2020 - Added the Grabbing of the Second Wobble Goal to Main Autonomous and Began Testing Main Autonomous for Upcoming Scrimmage.

- 12/8/2020 - Updated Side Autonomous Code for New Magazine and Shooter. Began Testing anf Tuning the Main Autonomous for Upcoming Scrimmage. Also, Began PID Control
Programming.

- 12/12/2020 - Updated Autonomous Programs and Post-Scrimmage Bug Fixes. We Hope to Get Main Auto Working for NC Scrimmage.

- 12/18/2020 - Updated Autonomous for New Wobble Goal Arm, Still Working on Grabbing Second Wobble Goal in Main Autonomous. Finished Two Positions of Grabbing, but Still 
Need to Work on Third Position. Hope to Finish by Jan 1.

- 12/22/2020 - Finished Fine-Tuning Two Wobble Goal Autonomous for Upcoming Competition Season. Also, Coded for New Inatake Being Built, for Faster Cycle Times.

- 12/25/2020 - Finished Autonomous for New Wobble Goal Arm and Full 71 Point Autonomous Done. Team Working on Refining Intake and Driver Practice.

- 12/26/2020 - Tweaked Autonomous and New Wobble Goal Arm for More Consistency and Speed. Future Updates Will Include Optimized Code for New Intake System.

- 1/1/2021 - Tweaked Autonomous with Better Odometry Positioning and Gyroscopic Turning. Made the 56 and 71 Point Autonomous Programs More Consistent and Efficient. Still
Working on Getting New Intake System Up and Running.

- 1/3/2021 - Fixed TeleOp Driving Bug and Improved the Driver Mechanum Controls for Smoother Movements. Continuing Work on Fixing Code for the Intake Control. Hope to Finish
All Subsystems and Code Operation Very Soon.

- 1/9/2021 - Fixed TeleOp Slow Mode Controls and the Controls for the New Intake System. Also, We Fixed the Autonomous for the IMU and For Improved Consistency.  

- 1/11/2021 - Fixed Controls for New Intake Mechanism and Finished Timing and Correcting Shooting and Turning in Autonomous. Thus, Completed 71-Point Autonomous.

- 1/12/2021 - Finalized and Formatted Code for Both Autonomous Programs and for the New Intake System. Hopefully, the New Code Will Make for a More Consistent Autonomous
and Faster TeleOp Cycles.

- 1/13/2021 - Optimized Subsystem Handshake for Better Speed and Cycle Times. Also, Updated Some Backup Vision Systems for Vision Implementation.