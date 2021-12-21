package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Systems.Core.GamePad;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@TeleOp (name="este año")
public class Test extends LinearOpMode {
    private Robot robot = new Robot();
    private GamePad driverPad;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, false);
        robot.mechanisms.initMechanisms(hardwareMap);
        driverPad = new GamePad(gamepad1);

        // Wait for the Play button to be pressed
        waitForStart();

        mainLoop: while (opModeIsActive()) {
            //Move Methods:
            moveRobot(); //may need to move these around if something needs to take precedence over something else
            moveSlides();
            moveCarousel();
            moveBasket();
            moveIntake();
            moveLevels();
        }
    }

    public void moveRobot() {
        //Sets the GamePad Values:
        driverPad.setGamePad();

        //Gets the Movement Variables:
        double x = driverPad.leftY;
        double y = driverPad.rightX;

        //Sets Motor Powers:
        robot.driveRobot(x, y, driverPad.leftBumper);

    }

    public void moveCarousel(){
        driverPad.setGamePad();

        if (driverPad.isRightBumperReleased()) {
            //Operates the Intake Arm:
            robot.mechanisms.operateFlywheel(robot.mechanisms.mainRPM);
        }
    }

    public void moveSlides(){
        //eventually using a color/distance sensor to automate the intake
        //the robot will hold the cube or sphere in the basket until the user wants to drop it
        //the operator will still need to tell the robot when it should drop the game element so it lands in the correct spot
        driverPad.setGamePad();

        if (driverPad.isXReleased()) {
            //Operates the Intake Arm:
            robot.mechanisms.operateSlides();
        }
    }

    public void moveBasket(){
        driverPad.setGamePad();

        if (driverPad.isYReleased()) {
            //Operates the basket
            robot.mechanisms.operateBasket();
        }
    }

    public void moveIntake(){
        driverPad.setGamePad();

        if (driverPad.isAReleased()) {
            //Operates the servo which will hit the game element into the basket
            robot.mechanisms.operateIntake();
        }
    }

    public void moveLevels(){
        driverPad.setGamePad();

        if (driverPad.isBReleased()) {
            //Operates the servo which will indicate which level the game element will fall into
            robot.mechanisms.operateLevels();
        }
    }
}


/*
* Color sensor code:
*
* ColorSensor colorSensor;
* colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor"); //inside runopmode
* telemetry.addData("Red", colorSensor.red()); //for the different colors
* telemetry.addData("Green", colorSensor.green());
* telemetry.addData("Blue", colorSensor.blue());
* telemetry.update();
*
*
*
* */