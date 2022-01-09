# Team 4653 Irrational DoorKeepers Team Code

This is the Code library for FTC Team #4653 Irrational DoorKeepers from Northern Guilford High School in Greensboro, NC.
This Repository Contains All of the Coding We Did, Including All Autonomous, TeleOp, and Calibration Testing. And....MAY THE FORCE BE WITH YOU!!!

## Change Log:

- Setup 2021-2022 season code.

- 12/20/2021: Modified the slide movement for the linear slides so it goes up to the appropriate height.
Coded the servos basketServo, intakeServo to move the basket and push the game elements into the basket.
Also coded levelsServo so when the linear slides bring the game element up to the drop zone, the angle of the ramp determines which level the element drops into.
Began coding the Autonomous.

- 12/28/2021: Worked on and finished the Park_Red and Park_Blue autonomous.
Realized the kinematic model from last year with the square robot was going to be strange with a triangle robot.
Adjusted the power of the motors because even with the added weight on the back of the robot, the back wheel would still jump when stopping.
Modified variables such as wheel diameter, wheelRPM, and various powers for the motors.
Sushen explained the vision and how it works to Matthew - basically taking the image of the phone camera and makes the screen smaller into areas small enough to make the vision very fast. Then the image is broken down into three areas of the image. Then the areas are analyzed for a specific rgb color and whichever area has the greatest number of a color indicates which position the game element is in.
Cleaned up some of the excess code from last year we didn't need anymore.

- 1/8/2022: Worked on phone vision using the squares on the phone screen displayed by the camera.
Changed the arrays of squareOne, squareTwo, and squareThree in the getBlockPosition() method.
Unfortunately after some testing and changing the color we are sensing for, and the margins acceptable for the color, I still found the phone not correctly detecting which position the duck/yellow color was at.