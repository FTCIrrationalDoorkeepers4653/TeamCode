package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Controller extends Positions {
  /* CONTROLLER VARIABLES */

  //Controller Objects:
  private static Robot robot = new Robot();
  private static ElapsedTime time = new ElapsedTime();

  //Controller Variables:
  private static double speed = 0.0;
  private static double constant = 0.0;
  private static double timeOut = 0.0;

  /* CONTROLLER SETUP METHODS */

  //Constructor:
  public Controller(double Kc, double power, double time) {
    //Sets the Constant:
    super();
    constant = Kc;
    speed = power;
    timeOut = time;
  }

  //Controller Motor Control Method:
  public static void applyController(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB) {
    //Gets Error Values:
    int initialLF = Math.abs(Math.abs(lF.getTargetPosition())
        - Math.abs(lF.getCurrentPosition()));
    int initialLB = Math.abs(Math.abs(lB.getTargetPosition())
        - Math.abs(lB.getCurrentPosition()));
    int initialRF = Math.abs(Math.abs(rF.getTargetPosition())
        - Math.abs(rF.getCurrentPosition()));
    int initialRB = Math.abs(Math.abs(rB.getTargetPosition())
        - Math.abs(rB.getCurrentPosition()));

    //Gets the Powers:
    double powerLF = controlPower(lF, initialLF);
    double powerLB = controlPower(lB, initialLB);
    double powerRF = controlPower(rF, initialRF);
    double powerRB = controlPower(rB, initialRB);

    //Sets the Powers:
    lF.setPower(powerLF);
    lB.setPower(powerLB);
    rF.setPower(powerRF);
    rB.setPower(powerRB);
    time.reset();
    time.startTime();

    //Loops for Power:
    mainLoop: while (lF.isBusy() || lB.isBusy() || rF.isBusy() || rB.isBusy()) {
      //Checks the Case:
      if (time.seconds() >= timeOut) {
        //Breaks the Loop:
        break mainLoop;
      }

      //Gets the Motor Powers:
      powerLF = controlPower(lF, initialLF);
      powerLB = controlPower(lB, initialLB);
      powerRF = controlPower(rF, initialRF);
      powerRB = controlPower(rB, initialRB);

      //Checks the Case:
      if (powerLF <= speed || powerLB <= speed || powerRF <= speed
          || powerRB <= speed) {
        //Gets and Sets the Power:
        powerLF = speed;
        powerLB = speed;
        powerRF = speed;
        powerRB = speed;
      }

      //Sets the Powers:
      lF.setPower(powerLF);
      lB.setPower(powerLB);
      rF.setPower(powerRF);
      rB.setPower(powerRB);
    }
  }

  /* CONTROL ALGORITHM METHODS */

  //Control Power Calculation Method (DcMotor):
  public static double controlPower(DcMotor motor, int initialDifference) {
    //Gets the Values:
    int current = Math.abs(motor.getCurrentPosition());
    int target = Math.abs(motor.getTargetPosition());

    //Calculates and Returns the Power:
    double error = controlError(current, target, initialDifference);
    double controlledPower = Math.abs(error * constant);
    return controlledPower;
  }

  //Control Error Method:
  public static double controlError(int current, int target, int initialDifference) {
    //Calculates and Returns Error:
    double error = Math.abs(Math.abs(target) - Math.abs(current));
    error /= Math.abs(initialDifference);
    return error;
  }
}