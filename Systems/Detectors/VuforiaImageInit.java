package org.firstinspires.ftc.teamcode.Systems.Detectors;

import android.graphics.Bitmap;
import android.graphics.Matrix;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

import java.util.ArrayList;

import lib.Analyze;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class VuforiaImageInit extends Analyze {
  //Hardware Map Object:
  HardwareMap hardwareMap;

  /* VUFORIA VARIABLES */

  //Vuforia Elements:
  VuforiaLocalizer vuforia;
  private static final String VUFORIA_KEY =
      "AR7KPuz/////AAABmSKvAg58mkBSqvvfxvaYqxMN8S2CvbOIzcpLyLVqb9hLPXQf3hPCERtF9azaj5sBUezFRBqdVA53ZAsNmlWW/ThqkaHtmpKNqXneP6p8VhN4liG3ofA7Cidx234PKNIhalLvby0jdmuxT5Uhh4dJjST6taoZGArAQz7Df8hzPG26Nd92L1ATW3mO4qzNAny2UK5YrzG92bUIxqvpDLkjeq8UNTLHYD4ulI1i+Jl/dPzU2PdeNPEqlsykdshGvcuRWRz8qeMXfpKVZ9TXmLxqvuTe6K291gxuKtfWXJ11rYJHTJlUAvooMpPaAh2/isv6LUy83+3UhIyl1kNxaNeMHK52iqEjpswOiOmVkniWTblp";
  private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

  //Image Variables:
  public static int globalWidth = 0, globalHeight = 0;

  /* VUFORIA METHODS */

  //Constructor:
  public VuforiaImageInit() {
    super();
  }

  //Initialize Vuforia:
  public void initVuforia(HardwareMap hwMap, int[] detectorRGBArray, String detectorName,
    int zoomValue, boolean flash) {
    /* Hardware Setup */

    //Declares Hardware Map:
    hardwareMap = hwMap;

    //Inits the Vuforia Localizer:
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    //Sets the Vuforia Elements:
    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraDirection = CAMERA_CHOICE;

    //Gets the Vuforia engine:
    vuforia = ClassFactory.getInstance().createVuforia(parameters);
    Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
    vuforia.setFrameQueueCapacity(1);

    /* Flash and Zoom Setup */

    //Flash and Zoom Value for Detection:
    com.vuforia.CameraDevice.getInstance().setFlashTorchMode(flash);
    String zoomSetting = Integer.toString(zoomValue);

    //Sets the Zoom Settings:
    com.vuforia.CameraDevice.getInstance().setField("opti-zoom", "opti-zoom-on");
    com.vuforia.CameraDevice.getInstance().setField("zoom", zoomSetting);

    /* Detector Setup */

    try {
      //Initializes the Custom Detector:
      initDetector(detectorName, detectorRGBArray[0], detectorRGBArray[1], detectorRGBArray[2]);
    }

    catch (Exception e) {
      e.printStackTrace();
    }
  }

  /* RGB METHODS */

  //Gets the Array of RGB Values from an Image:
  public int[][] getRGBArray(Bitmap image, int startX, int startY, int width, int height) {
    //Main RGB Array (w/ Default):
    int[][] rgbValues = new int[width][height];

    if (image != null) {
      //Gets RGB Values Array from Bitmap:
      int turnsWidth = startX;

      while (turnsWidth < width + startX) {
        int turnsHeight = startY;

        while (startY < height + startY) {
          rgbValues[turnsWidth - startX][turnsHeight - startY] = image.getPixel(turnsWidth, turnsHeight);

          turnsHeight++;
        }

        turnsWidth++;
      }
    }

    //Returns the Obtained RGB Array:
    return rgbValues;
  }

  /* DETECTOR METHODS */

  //RGB Comparison Method:
  public int[] getAverageRGBValues(int singleRGBValues[][]) {
    //Main Array:
    int localAverage[] = new int[3];

    try {
      //Computes the Average:
      localAverage = averageRGBValues(singleRGBValues);
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns RGB Average:
    return localAverage;
  }

  //Detection Boolean Method:
  public boolean detectObject(int[][] rgbValues, int[] lightingMargin, int pixelCount) {
    //Main Boolean:
    boolean mainBool = false;

    try {
      //Finds the Value Number of Pixels and Sets Boolean:
      int binaryValues[][] = binaryDetector(rgbValues, lightingMargin);
      mainBool = getBodyBoolean(binaryValues, pixelCount);
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns the Found Boolean:
    return mainBool;
  }

  //Detection Blob Method:
  public int detectBlobs(int[][] rgbValues, int[] lightingMargin, int distanceThreshold) {
    //Main Blob Count (w/ Default):
    int mainCount = 0;

    try {
      //Finds the Value Number of Pixels and Sets Boolean:
      int binaryValues[][] = binaryDetector(rgbValues, lightingMargin);
      mainCount = getBodyBlobs(binaryValues, distanceThreshold);
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns the Blob Count:
    return mainCount;
  }

  //Detection Pixel Method:
  public ArrayList<Integer> detectPixelCount(int[][] rgbValues, int[] lightingMargin,
                                             int distanceThreshold) {
    //Main Blob Pixel Count (w/ Default):
    ArrayList<Integer> pixelCounts = new ArrayList<Integer>();

    try {
      //Gets the Blob Detection:
      int blobCount = detectBlobs(rgbValues, lightingMargin, distanceThreshold);
      pixelCounts = getBlobPixelCounts();
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns the ArrayList:
    return pixelCounts;
  }

  //Boolean Pixel Detection Method:
  public int detectBooleanPixelCount(int[][] rgbValues, int[] lightingMargin, int pixels) {
    //Main Pixel Count Variable (w/ Default):
    int pixelCount = 0;

    try {
      //Gets the Boolean Pixel Count:
      boolean isThere = detectObject(rgbValues, lightingMargin, pixels);
      pixelCount = getBooleanPixelCount();
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    return pixelCount;
  }

  /* CAPTURE METHODS */

  //Vuforia Capture Image Method:
  public Bitmap getImage(double resizedRatio) {
    //PARAMS: Resized Ratio cannot be Zero!!!
    Frame frame;
    Bitmap bitmapImage;
    Bitmap finalImage = null;

    try {
      //Gets the Frame and Bitmap:
      frame = vuforia.getFrameQueue().take();
      bitmapImage = vuforia.convertFrameToBitmap(frame);

      //Resize the Bitmap:
      globalWidth = (int)(bitmapImage.getWidth() * resizedRatio);
      globalHeight = (int)(bitmapImage.getHeight() * resizedRatio);
      finalImage = Bitmap.createScaledBitmap(bitmapImage, globalWidth, globalHeight, true);
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns Image:
    return finalImage;
  }

  //Gets the Global Width:
  public static int getGlobalWidth() {
    //Returns the Width:
    return globalWidth;
  }

  //Gets the Global Height:
  public static int getGlobalHeight() {
    //Returns the Height:
    return globalHeight;
  }
}