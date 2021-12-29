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


        //Dropping element into correct shipping hub level

        //Parking
    }
}
