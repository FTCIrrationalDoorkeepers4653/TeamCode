package org.firstinspires.ftc.teamcode.Programs.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Systems.Core.Robot;

@Autonomous(name="Red")
public class Red extends LinearOpMode {
    /* MAIN AUTO VARIABLES */

    //Movement Variables:
    private Robot robot = new Robot();
    //Front of the Robot Variables
    //745-665 = 80 from front to back
    //92.37 from side to side
    private double startX = 665.0;
    private double startY = 320.0;
    private boolean camera = true;
    private boolean goForCarousel = false;

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
        telemetry.addData("Duck's at position", pos);
        telemetry.update();

        /* POSITION 1 */

        //low layer of the shipping hub
        if (pos == 1){
            //Move to the Shipping hub
            robot.mechanisms.intakeServo.setPosition(0.4);
            robot.mechanisms.turnGyro(-35.0, robot.slowPower, true);
            robot.mechanisms.runToPosition(575.0, 389.0, 1, 0.45, true);

            //Drop element into the correct level
            robot.mechanisms.automateSlides(0);
            robot.mechanisms.automateLevels(1);
            robot.mechanisms.automateBasket(0);
            robot.mechanisms.automateBasket(1);

            //If our teammates cannot go for the carousel points we can try
            //Otherwise go park
            if (goForCarousel){
                //broken for now
//                robot.mechanisms.turnGyro(87, robot.slowPower, true);
//                robot.mechanisms.automateFlywheel(1);
//                robot.mechanisms.runToPosition(675.0, 650.0, -1, robot.slowPower, true);
//                robot.mechanisms.automateSlides(1);
//                sleep(3000);
//                robot.mechanisms.automateFlywheel(0);

                //Go Park
//                robot.mechanisms.turnGyro(-10, robot.slowPower, true);
//                robot.mechanisms.runToPosition(573.0, 162.0, 1, robot.firePower, false);

            }   else {
                //Go park
                robot.mechanisms.intakeServo.setPosition(robot.mechanisms.intakeStartPosition);
                robot.mechanisms.turnGyro(-48, robot.slowPower, true);
                robot.mechanisms.runToPosition(579.0, 172.0, -1, robot.gyroPower, false);
                robot.mechanisms.automateSlides(1);
            }
        }


        /* POSITION 2 */

        //middle layer of the shipping hub
        else if (pos == 2){
            //Move to the shipping hub
            //Only change the next 2 lines
            robot.mechanisms.turnGyro(-41, robot.slowPower, true);
            robot.mechanisms.runToPosition(553.0, 427.0, 1, robot.gyroPower, true);

            //Drop element into the correct level
            robot.mechanisms.intakeServo.setPosition(0.5);
//            sleep(500);
            robot.mechanisms.automateSlides(0);
            robot.mechanisms.automateLevels(1);
            robot.mechanisms.automateBasket(0);
            robot.mechanisms.automateBasket(1);
            robot.mechanisms.intakeServo.setPosition(1.0);

            //If our teammates cannot go for the carousel points we can try
            //Otherwise go park
            if (goForCarousel){
                //broken for now
//                robot.mechanisms.turnGyro(87, robot.slowPower, true);
//                robot.mechanisms.automateFlywheel(1);
//                robot.mechanisms.runToPosition(675.0, 650.0, -1, robot.slowPower, true);
//                robot.mechanisms.automateSlides(1);
//                sleep(3000);
//                robot.mechanisms.automateFlywheel(0);

                //Go Park
//                robot.mechanisms.turnGyro(-10, robot.slowPower, true);
//                robot.mechanisms.runToPosition(573.0, 162.0, 1, robot.firePower, false);

            }   else {
                //Go park
                robot.mechanisms.turnGyro(-36, robot.slowPower, true);
                robot.mechanisms.runToPosition(579.0, 172.0, -1, robot.gyroPower, false);
                robot.mechanisms.automateSlides(1);
            }
        }


        /* POSITION 3 */

        //top layer of the shipping hub
        else {
            //Move to the shipping hub
            //Only change the next 2 lines
            robot.mechanisms.turnGyro(-40, robot.slowPower, true);
            robot.mechanisms.runToPosition(548.0, 432.0, 1, robot.gyroPower, true);

            //Drop element into the correct level
            robot.mechanisms.intakeServo.setPosition(0.5);
//            sleep(500);
            robot.mechanisms.automateSlides(0);
            robot.mechanisms.automateLevels(1);
            robot.mechanisms.automateBasket(0);
            robot.mechanisms.automateBasket(1);
            robot.mechanisms.intakeServo.setPosition(1.0);

            if (goForCarousel){
                //broken for now
//                robot.mechanisms.turnGyro(87, robot.slowPower, true);
//                robot.mechanisms.automateFlywheel(1);
//                robot.mechanisms.runToPosition(675.0, 650.0, -1, robot.slowPower, true);
////                robot.mechanisms.automateSlides(1);
//                sleep(3000);
//                robot.mechanisms.automateFlywheel(0);

                //Go Park
//                robot.mechanisms.turnGyro(-10, robot.slowPower, true);
//                robot.mechanisms.runToPosition(573.0, 162.0, 1, robot.firePower, false);

            }   else {
                //Go park
                robot.mechanisms.turnGyro(-37, robot.slowPower, true);
                robot.mechanisms.runToPosition(579.0, 172.0, -1, robot.gyroPower, false);
                robot.mechanisms.automateSlides(1);
            }
        }
    }
}
