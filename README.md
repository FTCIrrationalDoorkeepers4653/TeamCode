# Team 4653 Irrational DoorKeepers Team Code

This is the Code library for FTC Team #4653 Irrational DoorKeepers from Northern Guilford High School in Greensboro, NC.
This Repository Contains All of the Coding We Did, Including All Autonomous, TeleOp, and Calibration Testing. And....MAY THE FORCE BE WITH YOU!!!

## Change Log:

- Setup 2021-2022 season code.

- 12/20/2021: Modified the slide movement for the linear slides so it goes up to the appropriate height.
Coded the servos basketServo, intakeServo to move the basket and push the game elements into the basket.
Also coded levelsServo so when the linear slides bring the game element up to the drop zone, the angle of the ramp determines which level the element drops into.
Began coding the Autonomous.

- 12/28/2021:
Worked on and finished the Park_Red and Park_Blue autonomous.
Realized the kinematic model from last year with the square robot was going to be strange with a triangle robot.
Adjusted the power of the motors because even with the added weight on the back of the robot, the back wheel would still jump when stopping.
Modified variables such as wheel diameter, wheelRPM, and various powers for the motors.
Sushen explained the vision and how it works to Matthew - basically taking the image of the phone camera and makes the screen smaller into areas small enough to make the vision very fast. Then the image is broken down into three areas of the image. Then the areas are analyzed for a specific rgb color and whichever area has the greatest number of a color indicates which position the game element is in.
Cleaned up some of the excess code from last year we didn't need anymore.

- 1/8/2022:
Worked on phone vision using the squares on the phone screen displayed by the camera.
Changed the arrays of squareOne, squareTwo, and squareThree in the getBlockPosition() method.
Unfortunately after some testing and changing the color we are sensing for, and the margins acceptable for the color, I still found the phone not correctly detecting which position the duck/yellow color was at.

- 1/9/2022:
After some initial testing from the code used yesterday, I found the color correctly detected which position the duck was at when the flash of the phone turned on, but not when I not when the play button was pushed directly after the init button was pressed.
I tried to work on the autonomous but when I tried to work with the levelsServo and the linear slides, they would not work even though they were instructed to.
I think the problem was coming from how there was no wait time in the code, so everything was trying to execute at the same time.

- 1/15/2022:
Using the AutonomousMapping Program, I started working on the positioning movements for the robot.
I did realize that the bot was moving a little farther than it should have been, so I changed the startX and startY position to reflect the start of the bot instead of the back.
Just moved the robot to the correct position close enough to the shipping hub to drop the cube into one of the levels.

- 1/16/2022:
After changing the startX and startY positions to the correct location at the front of the bot, I started with the high goal autonomous:
Turned, moved forward to the correct position, used the intake as a claw to hook the wobble goal.
The linear slides were broken at the time so there was no work done with the slides, the basket, or dropping the cube into the shipping hub.

- 1/17/2022:
After positioning with the high goal worked, the middle goal was operated next.
Fundamentally the robot did the same tasks, except it didn't go as far forward and the intake didn't go down.
Did work on the engineering for the linear slides, so testing was done at a slow speed to make sure the slides worked consistently and fell when going down.

- 1/18/2022:
The high and middle goal positions were working at this point. Starting to work on the low level.
At some point when going for the lower goal, after dropping the cube, sometimes because the window was so small the cube would hit the floor and bounce into the low goal.
I started trying to automate some of the TeleOp functions so it would be faster and more efficient when driving. Can be seen in the operateEverything() method.

- 1/22/2022:
Automated the intake function so when the automation button is pressed, the robot consistently picks up a block or a ball into the basket.
Increased the power to the linear slides to they operate a little faster.

- 1/29/2022:
Changed the sleep timers to the autonomous functions so during TeleOp, the driver is able to still move around while mechanisms are moving.
Fine tuned all of the sleep times so there isn't much unnecessary waiting.
Further automated the linear slides and basket dropping to when the driver is close to the shipping hub. 
