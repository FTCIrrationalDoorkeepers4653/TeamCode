package org.firstinspires.ftc.teamcode.Systems.Vision;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Systems.Vision.Lib.Camera;

import java.util.ArrayList;
import java.util.List;

public class TensorVision extends Camera {
  /* TENSORFLOW VARIABLES */

  //TensorFlow Object:
  public static TFObjectDetector tfod;

  //TensorFlow Settings:
  public static String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
  public static String LABEL_FIRST_ELEMENT = "Quad";
  public static String LABEL_SECOND_ELEMENT = "Single";

  //TensorFlow Detection Outputs:
  public static ArrayList<String> detections = new ArrayList<String>();
  public static ArrayList<Float> top = new ArrayList<Float>();
  public static ArrayList<Float> bottom = new ArrayList<Float>();
  public static ArrayList<Float> left = new ArrayList<Float>();
  public static ArrayList<Float> right = new ArrayList<Float>();

  /* TENSORFLOW METHODS */

  //Constructor:
  public TensorVision() {
    super();
  }

  //Initializes the TensorFlow Model:
  public static void initTensorVision() {
    //Initializes the TensorFlow Parameters:
    int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
        "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    tfodParameters.minResultConfidence = 0.8f;
    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
  }

  //TensorFlow Object Detection:
  public static void detectTensorVision() {
    //Checks the Case:
    if (tfod != null) {
      //Starts Object Detection:
      tfod.activate();
    }

    //Gets the List of Detections:
    List<Recognition> recognitions = tfod.getUpdatedRecognitions();

    //Checks the Case:
    if (recognitions != null) {
      //Loops through Detections:
      for (Recognition recognition : recognitions) {
        //Adds the Detection Info:
        detections.add(recognition.getLabel());
        top.add(recognition.getTop());
        bottom.add(recognition.getBottom());
        left.add(recognition.getLeft());
        right.add(recognition.getRight());
      }
    }

    //Checks the Case:
    if (tfod != null) {
      //Stops Object Detection:
      tfod.deactivate();
    }
  }

  /* UTILITY METHODS */

  //Get Detections Method:
  public static ArrayList<String> getDetections() {
    //Returns Detections:
    return detections;
  }

  //Get Tops Method:
  public static ArrayList<Float> getTops() {
    //Returns Tops:
    return top;
  }

  //Get Bottoms Method:
  public static ArrayList<Float> getBottoms() {
    //Returns Bottoms:
    return bottom;
  }

  //Get Lefts Method:
  public static ArrayList<Float> getLefts() {
    //Returns Lefts:
    return left;
  }

  //Get Rights Method:
  public static ArrayList<Float> getRights() {
    //Returns Rights:
    return right;
  }
}
