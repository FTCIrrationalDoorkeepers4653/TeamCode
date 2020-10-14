package org.firstinspires.ftc.teamcode.Systems.Vision;

import android.graphics.Bitmap;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@SuppressWarnings("unused")
public class Camera extends Vision {
  /* VUFORIA VARIABLES */

  //Hardware Map Object:
  static HardwareMap hardwareMap;

  //Vuforia Elements:
  static VuforiaLocalizer vuforia;
  static VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
  public static int globalWidth = 0, globalHeight = 0;

  /* VUFORIA METHODS */

  //Constructor:
  public Camera() {
    super();
  }

  //Initialize Vuforia:
  public static void initVuforia(HardwareMap hwMap, String API_KEY, int zoomValue, boolean flash) {
    /* Hardware Setup */

    //Declares Hardware Map:
    hardwareMap = hwMap;

    //Inits the Vuforia Localizer:
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    //Sets the Vuforia Elements:
    parameters.vuforiaLicenseKey = API_KEY;
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
  }

  /* IMAGE METHODS */

  //Vuforia Capture Image Method:
  public static Bitmap getImage(double resizeRatio) {
    //Main Bitmap Image:
    Bitmap image = null;

    try {
      //Takes Image Using Closeable Frame:
      VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
      Image rgb = null;
      Bitmap unscaled;

      //Loops through Frames:
      for (int i = 0; i < frame.getNumImages(); i++) {
        //Checks for Correct RGB Format:
        if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
          //Stores the Image:
          rgb = frame.getImage(i);
          break;
        }
      }

      //Closes, Converts, and Resizes:
      frame.close();
      unscaled = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
      unscaled.copyPixelsFromBuffer(rgb.getPixels());
      image = resizeBitmap(unscaled, resizeRatio);
    }

    catch (Exception e) {
      e.printStackTrace();
    }

    //Returns Image:
    return image;
  }

  //Bitmap Scaling Method:
  public static Bitmap resizeBitmap(Bitmap image, double resizeRatio) {
    //Gets the Width and Height:
    globalWidth = (int)(image.getWidth() * resizeRatio);
    globalHeight = (int)(image.getHeight() * resizeRatio);

    //Resizes Image and Returns:
    Bitmap scaled = Bitmap.createScaledBitmap(image, globalWidth, globalHeight, true);
    return scaled;
  }

  /* UTILITY METHODS */

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
