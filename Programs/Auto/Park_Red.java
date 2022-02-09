package org.firstinspires.ftc.teamcode.Programs.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@Autonomous(name="Park_Red")
public class Park_Red extends LinearOpMode {
    /* MAIN AUTO VARIABLES */

    //Movement Variables:
    private Robot robot = new Robot();
    //Front of the Robot Variables
    //745-665 = 80 from front to back
    private double startX = 665.0;
    private double startY = 320.0;
    private boolean camera = false;

    /* OPMODE METHODS */

    @Override
    public void runOpMode() {
        /* Initialization */

        //Hardware Initialization:
        robot.init(hardwareMap, camera);
        robot.mechanisms.initMechanisms(hardwareMap);
        robot.mechanisms.setCurrentPosition(startX, startY, robot.getTheta());

        //Waits for Start:
        waitForStart();

        //broken now
        //Moves forward to park, Turns, and Parks:
        robot.mechanisms.runToPosition(620.0, 320.0, 1, robot.firePower, false);
        robot.mechanisms.turnGyro(90.0, robot.firePower, false);
        robot.mechanisms.runToPosition(620.0, 100.0, 1, robot.firePower, false);
    }
}