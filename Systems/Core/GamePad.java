package org.firstinspires.ftc.teamcode.Systems.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamePad {
  /* GAMEPAD MAIN VARIABLES */

  //Main GamePad Variables:
  private Gamepad gamepad = new Gamepad();
  private Robot robot = new Robot();

  /* GAMEPAD INTERFACE VARIABLES */

  //Control Button Variables:
  public boolean aButton = false;
  public boolean bButton = false;
  public boolean xButton = false;
  public boolean yButton = false;

  //Joystick Variables:
  public double leftX = 0.0;
  public double leftY = 0.0;
  public double rightX = 0.0;
  public double rightY = 0.0;
  public boolean leftJoyButton = false;
  public boolean rightJoyButton = false;

  //Trigger Variables:
  public double leftTrigger = 0.0;
  public double rightTrigger = 0.0;
  public boolean leftBumper = false;
  public boolean rightBumper = false;

  //D-Pad and Button Variables:
  public boolean dPadUp = false;
  public boolean dPadDown = false;
  public boolean dPadLeft = false;
  public boolean dPadRight = false;

  /* GAMEPAD TRACKER VARIABLES */

  //Control Joystick Tracker Variables:
  private boolean wasLeftJoyPressed = false;
  private boolean wasRightJoyPressed = false;

  //Control Triggers Tracker Variables:
  private boolean wasLeftBumperPressed = false;
  private boolean wasRightBumperPressed = false;

  //Control Buttons Tracker Variables:
  private boolean wasAPressed = false;
  private boolean wasBPressed = false;
  private boolean wasXPressed = false;
  private boolean wasYPressed = false;

  /* GAMEPAD SETUP */

  //Constructor:
  public GamePad(Gamepad gamepad) {
    super();
    this.gamepad = gamepad;
  }

  //Sets the GamePad Values:
  public void setGamePad() {
    //Control Buttons:
    aButton = gamepad.a;
    bButton = gamepad.b;
    xButton = gamepad.x;
    yButton = gamepad.y;

    //Joysticks:
    leftX = gamepad.left_stick_x;
    leftY = gamepad.left_stick_y;
    rightX = gamepad.right_stick_x;
    rightY = gamepad.right_stick_y;
    leftJoyButton = gamepad.left_stick_button;
    rightJoyButton = gamepad.right_stick_button;

    //Triggers:
    leftTrigger = gamepad.left_trigger;
    rightTrigger = gamepad.right_trigger;
    leftBumper = gamepad.left_bumper;
    rightBumper = gamepad.right_bumper;

    //D-Pad:
    dPadUp = gamepad.dpad_up;
    dPadDown = gamepad.dpad_down;
    dPadLeft = gamepad.dpad_left;
    dPadRight = gamepad.dpad_right;
  }

  /* GAMEPAD JOYSTICK METHODS */

  //Left Joystick Button Released:
  public boolean isLeftJoyButtonReleased() {
    //Checks the Case:
    if (leftJoyButton) {
      //Sets the Button Press:
      wasLeftJoyPressed = true;
    }

    //Checks the Case:
    if (wasLeftJoyPressed && !leftJoyButton) {
      //Returns the Value:
      wasLeftJoyPressed = false;
      return true;
    }

    else {
      //Returns the Value:
      return false;
    }
  }

  //Right Joystick Button Released:
  public boolean isRightJoyButtonReleased() {
    //Checks the Case:
    if (rightJoyButton) {
      //Sets the Button Press:
      wasRightJoyPressed = true;
    }

    //Checks the Case:
    if (wasRightJoyPressed && !rightJoyButton) {
      //Returns the Value:
      wasRightJoyPressed = false;
      return true;
    }

    else {
      //Returns the Value:
      return false;
    }
  }

  /* GAMEPAD TRIGGER METHODS */

  //Left Trigger Released:
  public boolean isLeftBumperReleased() {
    //Checks the Case:
    if (leftBumper) {
      //Sets the Button Press:
      wasLeftBumperPressed = true;
    }

    //Checks the Case:
    if (wasLeftBumperPressed && !leftBumper) {
      //Returns the Value:
      wasLeftBumperPressed = false;
      return true;
    }

    else {
      //Returns the Value:
      return false;
    }
  }

  //Right Trigger Released:
  public boolean isRightBumperReleased() {
    //Checks the Case:
    if (rightBumper) {
      //Sets the Button Press:
      wasRightBumperPressed = true;
    }

    //Checks the Case:
    if (wasRightBumperPressed && !rightBumper) {
      //Returns the Value:
      wasRightBumperPressed = false;
      return true;
    }

    else {
      //Returns the Value:
      return false;
    }
  }

  /* GAMEPAD BUTTON METHODS */

  //A Button Released:
  public boolean isAReleased() {
    //Checks the Case:
    if (aButton) {
      //Sets the Button Press:
      wasAPressed = true;
    }

    //Checks the Case:
    if (wasAPressed && !aButton) {
      //Returns the Value:
      wasAPressed = false;
      return true;
    }

    else {
      //Returns the Value:
      return false;
    }
  }

  //B Button Released:
  public boolean isBReleased() {
    //Checks the Case:
    if (bButton) {
      //Sets the Button Press:
      wasBPressed = true;
    }

    //Checks the Case:
    if (wasBPressed && !bButton) {
      //Returns the Value:
      wasBPressed = false;
      return true;
    }

    else {
      //Returns the Value:
      return false;
    }
  }

  //X Button Released:
  public boolean isXReleased() {
    //Checks the Case:
    if (xButton) {
      //Sets the Button Press:
      wasXPressed = true;
    }

    //Checks the Case:
    if (wasXPressed && !xButton) {
      //Returns the Value:
      wasXPressed = false;
      return true;
    }

    else {
      //Returns the Value:
      return false;
    }
  }

  //Y Button Released:
  public boolean isYReleased() {
    //Checks the Case:
    if (yButton) {
      //Sets the Button Press:
      wasYPressed = true;
    }

    //Checks the Case:
    if (wasYPressed && !yButton) {
      //Returns the Value:
      wasYPressed = false;
      return true;
    }

    else {
      //Returns the Value:
      return false;
    }
  }
}
