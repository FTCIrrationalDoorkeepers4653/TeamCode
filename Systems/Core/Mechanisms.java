package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Mechanisms extends Positions {
  /* MECHANISMS SETUP VARIABLES */

  //Robot Object:
  private static Robot robot = new Robot();

  //Motor Mechanisms:
  public static DcMotor slideMotor;
  public static DcMotor carouselMotor;

  //Servo Mechanisms:
  public static Servo basketServo;
  public static Servo intakeServo;
  public static Servo levelsServo;

  /* MECHANISMS ARM CONTROL VARIABLES */

  //Linear Slide Variables:
  public static int slideMovement = -640;
  public static int slide = 0;

  //Bucket Variables:
  public static double basketUpPosition = 0.2;
  public static double basketDownPosition = 0.0;
  public static int basket = 0;

  //Intake Servo Variables:
  public static double intakeStartPosition = 0.0; //change these later
  public static double intakeEndPosition = 1.0; //change these later
  public static int intake = 0;

  //Positions for the levels of the shipping hub
  public static double levelsInitPosition = 0.3;
  public static double levelsHigh = 0.2;
  public static double levelsMid = 0.1;
  public static double levelsLow = 0.0;
  public static int levels = 0;

  /* MECHANISM SHOOTER CONTROL VARIABLES */

  //Mechanism Flywheel Variables:
  public static int shooter = 0;

  /* MECHANISMS INITIALIZATION METHODS */
  //Constructor:
  public Mechanisms() {
    super();
  }

  //Initializes the Mechanisms:
  public static void initMechanisms(HardwareMap hardwareMap) {
    /* Initialization */

    //Motor Mechanism Maps:
    slideMotor = hardwareMap.dcMotor.get("slideMotor");
    carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");

    //Servo Mechanism Maps:
    basketServo = hardwareMap.servo.get("basketServo");
    intakeServo = hardwareMap.servo.get("intakeServo");
    levelsServo = hardwareMap.servo.get("levelsServo");

    //Servo Mechanism Setup:
    basketServo.setPosition(basketUpPosition);
    intakeServo.setPosition(intakeStartPosition);
    levelsServo.setPosition(levelsInitPosition);

    /* Setup */

    //Mechanism Motors Behavior:
    slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    //Mechanism Motors Direction:
    slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    carouselMotor.setDirection(DcMotor.Direction.REVERSE);

    //Mechanism Motors Encoders Reset:
    carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //Mechanism Motors Encoders:
    slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    //Mechanism Motors Powers:
    slideMotor.setPower(robot.zeroPower);
    carouselMotor.setPower(robot.zeroPower);
  }

  /* MECHANISM OPERATION METHODS */

  //Operates the Flywheel:
  public void operateFlywheel() {
    //Checks the Case:
    if (shooter == 0) {
      //Sets the Motor:
      carouselMotor.setPower(robot.zeroPower);
      shooter++;
    }

    else if (shooter == 1) {
      //Sets the Motor:
      carouselMotor.setPower(robot.slowPower);
      shooter--;
    }
  }

  //Operates the Intake Arm:
  public void operateIntake() {
    //Checks the Case:
    if (intake == 0) {
      //Runs the Intake:
      intakeServo.setPosition(intakeStartPosition);
      intake++;
    }

    else if (intake == 1) {
      //Runs the Intake:
      intakeServo.setPosition(intakeEndPosition);
      intake--;
    }
  }

  //method to operate the basket and dump the game element into the top compartment
  public void operateBasket() {
    if (basket == 0) {
      basketServo.setPosition(basketDownPosition);
      basket++;
    }

    else if (basket == 1) {
      basketServo.setPosition(basketUpPosition);
      basket--;
    }
  }

  //Linear slide operation method
  public void operateSlides() {
    if (slide == 0) {
      slideMotor.setTargetPosition(slideMotor.getCurrentPosition() + slideMovement);
      slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      slideMotor.setPower(robot.slowPower);
      slide++;
    }

    else if (slide == 1) {
      slideMotor.setTargetPosition(slideMotor.getCurrentPosition() - slideMovement);
      slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      slideMotor.setPower(robot.slowPower);
      slide--;
    }
  }

  public void operateLevels() {
    //Checks the Case:
    if (levels == 0) {
      //Runs the Intake:
      levelsServo.setPosition(levelsHigh);
      levels++;
    }

    else if (levels == 1) {
      //Runs the Intake:
      levelsServo.setPosition(levelsMid);
      levels++;
    }

    else if (levels == 2) {
      //Runs the Intake:
      levelsServo.setPosition(levelsLow);
      levels -= 2;
    }
  }
}