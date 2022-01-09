package org.firstinspires.ftc.teamcode.Programs.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@Autonomous(name="Red")
public class Red extends LinearOpMode {
    /* MAIN AUTO VARIABLES */

    //Movement Variables:
    private Robot robot = new Robot();
    private double startX = 16.0;
    private double startY = 320.0;
    private boolean camera = true;

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

        //Detect the Ducks location
        int pos = robot.getBlockPosition();
        telemetry.addData("Position: ", pos);
        telemetry.update();

//        robot.mechanisms.runToPosition(700.0, 320.0, 1, robot.firePower, false);
//
//        if (pos == 1){
//            robot.mechanisms.turnGyro(-90.0, robot.firePower, false);
//            robot.mechanisms.runToPosition(700.0, 100.0, 1, robot.firePower, false);
//        } else if (pos == 2){
//            robot.mechanisms.runToPosition(730.0, 320.0, 1, robot.firePower, false);
//        } else if (pos == 3){
//            robot.mechanisms.turnGyro(90.0, robot.firePower, false);
//            robot.mechanisms.runToPosition(700.0, 400.0, 1, robot.firePower, false);
//        }

        //Dropping element into correct shipping hub level

        //Parking
    }
}
