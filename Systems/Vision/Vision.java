package org.firstinspires.ftc.teamcode.Systems.Vision;

import android.graphics.Bitmap;
import java.util.ArrayList;
import lib.Analyze;
import lib.ImageRecognition;

@SuppressWarnings("unused")
public class Vision extends Analyze {
  /* VISION VARIABLES */

  //YOLO Detection Information Variables:
  private static ArrayList<Integer> detectionX = new ArrayList<Integer>();
  private static ArrayList<Integer> detectionY = new ArrayList<Integer>();
  private static ArrayList<Integer> detectionWidth = new ArrayList<Integer>();
  private static ArrayList<Integer> detectionHeight = new ArrayList<Integer>();

  /* SETUP METHODS */

  //Constructor:
  public Vision() {
    super();
  }

  //Detector Setup:
  public static void initDetector(String detectorName, int rgb[]) {
    try {
      //Detector Setup:
      initDetector(detectorName, rgb[0], rgb[1], rgb[2]);
    }

    catch (Exception e) {
      e.printStackTrace();
    }
  }

  /* RGB METHODS */

  //Gets the Array of RGB Values from an Image:
  public static int[][] getBitmapRGB(Bitmap image, int startX, int startY, int width, int height) {
    //Main RGB Array (w/ Default):
    int[][] rgbValues = new int[width][height];

    if (image != null && startX + width <= image.getWidth() && startY + height <= image.getHeight()) {
      //Gets RGB Values Array from Bitmap:
      int turnsWidth = startX;

      //Loops through Array:
      mainLoop: while (turnsWidth < startX + width) {
        //Loop Counter:
        int turnsHeight = startY;

        //Loops through Array:
        secondLoop: while (turnsHeight < startY + height) {
          //Sets the RGB Array Values:
          rgbValues[turnsWidth - startX][turnsHeight - startY] = image.getPixel(turnsWidth, turnsHeight);

          turnsHeight++;
        }

        turnsWidth++;
      }
    }

    //Returns the Obtained RGB Array:
    return rgbValues;
  }

  //Bitmap from RGB Method:
  public static Bitmap rgbBitmap(int rgb[][]) {
    //Creates Bitmap:
    Bitmap image = Bitmap.createBitmap(rgb.length, rgb[0].length, Bitmap.Config.RGB_565);

    //Loop Variable:
    int turnsWidth = 0;

    //Loops through Array:
    mainLoop: while (turnsWidth < rgb.length) {
      //Loop Variable:
      int turnsHeight = 0;

      //Loops through Array:
      secondLoop: while (turnsHeight < rgb[0].length) {
        //Sets the Pixel:
        image.setPixel(turnsWidth, turnsHeight, rgb[turnsWidth][turnsHeight]);

        turnsHeight++;
      }

      turnsWidth++;
    }

    //Returns the Image:
    return image;
  }

  /* MACHINE LEARNING METHODS */

  //YOLO Detection Method (Pre-Trained):
  public static int trackYOLO(Bitmap image, String identifier, int width, int height, int vectorGrid, double authPercent, int boundings) {
    //Main Detection Integer (w/ Default):
    int detections = 0;

    try {
      //Gets the Image:
      Bitmap capture = Bitmap.createScaledBitmap(image, width, height, true);
      int authRGB[][] = getBitmapRGB(capture, 0, 0, capture.getWidth(), capture.getHeight());

      //Loop Variable:
      int turns = 0;

      //Loops through Boundings:
      mainLoop:
      while (turns < boundings) {
        //Gets Random Starting Points:
        int localX = (int) (randomDouble(0, authRGB.length - 1));
        int localY = (int) (randomDouble(0, authRGB[0].length - 1));
        int localWidth = (int) (randomDouble(localX, authRGB.length - 1));
        int localHeight = (int) (randomDouble(localY, authRGB[0].length - 1));

        //Gets the Section of RGB and Converts:
        int section[][] = getSection(authRGB, localX, localY, localWidth, localHeight);
        Bitmap unscaled = rgbBitmap(section);
        Bitmap scaled = Bitmap.createScaledBitmap(unscaled, width, height, true);

        //Checks the Case:
        if (ImageRecognition.authenticateImage(identifier, authRGB, 0, 0, authPercent, vectorGrid)) {
          //Adds a Detection:
          detections++;

          //Adds to ArrayLists:
          detectionX.add(localX);
          detectionY.add(localY);
          detectionWidth.add(localWidth);
          detectionHeight.add(localHeight);
        }

        turns++;
      }
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns the Detection:
    return detections;
  }

  //Object Detection Tracking Method:
  public static boolean trackObject(int rgbValues[][], String identifier, int grid,
    double percentage) {
    //Gets the Object in RGB Values:
    boolean isThere = false;

    try {
      //Gets the Object Detection:
      isThere = ImageRecognition.authenticateImage(identifier, rgbValues,
        0, 0, percentage, grid);
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns the Detection:
    return isThere;
  }

  //Object Training Method:
  public static void trainObject(int rgbValues[][], String identifier, int grid) {
    try {
      //Trains a Model on RGB Values:
      ImageRecognition.trainImage(identifier, rgbValues, 0, 0,
        0, grid);
    }

    catch (Exception e) {
      e.printStackTrace();
    }
  }

  /* DETECTOR METHODS */

  //RGB Comparison Method:
  public static int[] getAverageRGBValues(int rgbValues[][]) {
    //Main Array:
    int localAverage[] = new int[3];

    try {
      //Computes the Average:
      localAverage = averageRGBValues(rgbValues);
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns RGB Average:
    return localAverage;
  }

  //Detection Boolean Method:
  public static boolean detect(int[][] rgbValues, int[] lightingMargin, int pixelCount) {
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
  public static int detectBlobs(int[][] rgbValues, int[] lightingMargin, int distanceThreshold) {
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

  /* PIXEL COUNT METHODS */

  //Pixel Detection Method:
  public static int detectPixelCount(int[][] rgbValues, int[] lightingMargin, int pixels) {
    //Main Pixel Count Variable (w/ Default):
    int pixelCount = 0;

    try {
      //Gets the Boolean Pixel Count:
      boolean isThere = detect(rgbValues, lightingMargin, pixels);
      pixelCount = getBooleanPixelCount();
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    return pixelCount;
  }

  //Blob Detection Pixel Method:
  public static ArrayList<Integer> detectBlobsPixelCount(int[][] rgbValues, int[] lightingMargin,
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

  /* COORDINATE METHODS */

  //Get All Blob Coordinates Method:
  public static int[][] getCoordinates(int[][] rgbValues, int[] lightingMargin,
    int distanceThreshold) {
    //Main Coordinates Array (w/ Default):
    int coordinates[][] = null;

    try {
      //Gets the Blob Detection:
      int blobCount = detectBlobs(rgbValues, lightingMargin, distanceThreshold);
      ArrayList<Integer> x = getBlobX();
      ArrayList<Integer> y = getBlobY();
      coordinates = new int[2][blobCount];

      //Checks the Case:
      if (x.size() == y.size() && blobCount == x.size() && blobCount == y.size()) {
        //Loop Variable:
        int turns = 0;

        //Loops through Array:
        mainLoop: while (turns < x.size()) {
          //Sets the Coordinates:
          coordinates[0][turns] = x.get(turns);
          coordinates[1][turns] = y.get(turns);

          turns++;
        }
      }
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns the Coordinates:
    return coordinates;
  }

  /* UTILITY METHODS */

  //Gets the YOLO Detection X:
  public static ArrayList<Integer> getDetectionX() {
    //Returns the Detection X:
    return detectionX;
  }

  //Gets the YOLO Detection Y:
  public static ArrayList<Integer> getDetectionY() {
    //Returns the Detection Y:
    return detectionY;
  }

  //Gets the YOLO Detection Width:
  public static ArrayList<Integer> getDetectionWidth() {
    //Returns the Detection Width:
    return detectionWidth;
  }

  //Gets the YOLO Detection Height:
  public static ArrayList<Integer> getDetectionHeight() {
    //Returns the Detection Height:
    return detectionHeight;
  }
}