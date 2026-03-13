package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class CameraConstants {
  public static final boolean kReadEnabledByDefault = true;

  // Processing camera (AprilTag detection) on USB index 1.
  public static final UsbCameraConfig kAprilTagCamera =
      new UsbCameraConfig("AprilTagCam", 0, 640, 480, 15);

  // Driver camera feed on USB index 0.
  public static final UsbCameraConfig kDriverCamera =
      new UsbCameraConfig("DriverCam", 1, 320, 240, 12);

  // Both USB cameras are mounted upside down, so the published streams and
  // AprilTag processing frames are rotated 180 degrees in software.
  public static final boolean kRotateAprilTagCamera180 = true;
  public static final boolean kRotateDriverCamera180 = true;

  // CPU-friendly vision processing rate. The vision thread sleeps between updates.
  public static final double kVisionProcessHz = 12.0;

  // Minimum decision margin to accept a detection for distance/pose updates.
  public static final double kMinDecisionMargin = 10.0;

  // AprilTag physical size in meters (default FRC tag size is 6.5 in).
  public static final double kAprilTagSizeMeters = 0.1651;

  // Approximate intrinsics for LifeCam HD-3000 at 640x480.
  // Replace with calibration values for best accuracy.
  public static final double kFxPixels = 554.0;
  public static final double kFyPixels = 554.0;
  public static final double kCxPixels = 320.0;
  public static final double kCyPixels = 240.0;

  // Manual image settings for the AprilTag camera to reduce motion blur and glare.
  public static final int kAprilTagBrightness = 35;
  public static final int kAprilTagExposure = 30;

  // Detector tuning for smaller/farther tags.
  public static final int kAprilTagDetectorThreads = 2;
  public static final float kAprilTagQuadDecimate = 1.0f;
  public static final float kAprilTagQuadSigma = 0.0f;
  public static final boolean kAprilTagRefineEdges = true;
  public static final double kAprilTagDecodeSharpening = 0.5;
  public static final int kAprilTagMinClusterPixels = 150;
  public static final int kAprilTagMinWhiteBlackDiff = 3;
  public static final boolean kAprilTagDeglitch = false;

  // Empirical correction on top of the corner-size estimate.
  // Fit from measured camera distance -> real field distance:
  // 1.78 -> 1.8288, 2.47 -> 2.7432, 3.16 -> 3.6576, 4.06 -> 4.572
  public static final double kAprilTagDistanceSlope = 1.2087;
  public static final double kAprilTagDistanceInterceptMeters = -0.2656;

  // Robot-to-camera transform in robot coordinates.
  // Update for your real mount location and orientation.
  public static final Transform3d kRobotToAprilTagCamera =
      // forward/backward, left/right, up/down from ground, then camera rotation
      new Transform3d(
          new Translation3d(0.0127, 0.0762, 0.5588),
          kRotateAprilTagCamera180 ? new Rotation3d(Math.PI, 0.0, 0.0) : new Rotation3d());

  private CameraConstants() {}

  public record UsbCameraConfig(String name, int deviceIndex, int width, int height, int fps) {}
}
