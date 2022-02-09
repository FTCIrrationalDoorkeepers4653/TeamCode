package org.firstinspires.ftc.teamcode.Programs.Driver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Systems.Core.GamePad;
import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@TeleOp (name="este año")
public class Driver extends LinearOpMode {
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
            moveEverything();
        }
    }

    //Moves the Robot:
    public void moveRobot() {
        //Sets the GamePad Values:
        driverPad.setGamePad();

        //Gets the Movement Variables:
        double x = driverPad.leftY;
        double y = driverPad.rightX;

        //Sets Motor Powers:
        robot.driveRobot(x, y, driverPad.leftBumper);

    }

    //Moves the Carousel:
    public void moveCarousel() {
        driverPad.setGamePad();

        if (driverPad.isDpadDownReleased()) {
            //Operates the Intake Arm:
            robot.mechanisms.operateFlywheel();
        }
    }

    //Moves the Intake:
    public void moveIntake() {
        driverPad.setGamePad();

        if (driverPad.isXReleased()) {
            //Operates the servo which will hit the game element into the basket
            robot.mechanisms.operateIntake();
        }
    }

    //Moves the Basket:
    public void moveBasket() {
        driverPad.setGamePad();

        if (driverPad.isAReleased()) {
            //Operates the basket
            robot.mechanisms.operateBasket();
        }
    }

    //Moves the Slides:
    public void moveSlides() {
        //the robot will hold the cube or sphere in the basket until the user wants to drop it
        //the operator will still need to tell the robot when it should drop the game element so it lands in the correct spot
        driverPad.setGamePad();

        if (driverPad.isBReleased()) {
            //Operates the Intake Arm:
            robot.mechanisms.operateSlides();
        }
    }

    //Moves the Levels:
    public void moveLevels() {
        driverPad.setGamePad();

        if (driverPad.isYReleased()) {
            //Operates the servo which will indicate which level the game element will fall into
            robot.mechanisms.operateLevels();
        }
    }

    //Driver automation method:
    public void moveEverything() {
        driverPad.setGamePad();

        if (driverPad.isRightBumperReleased()) {
            robot.mechanisms.operateEverything();
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