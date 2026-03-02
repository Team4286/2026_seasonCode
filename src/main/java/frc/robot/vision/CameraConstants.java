package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class CameraConstants {
  public static final boolean kReadEnabledByDefault = true;

  // Processing camera (AprilTag detection) on USB index 1.
  public static final UsbCameraConfig kAprilTagCamera =
      new UsbCameraConfig("AprilTagCam", 1, 320, 240, 12);

  // Driver camera feed on USB index 0.
  public static final UsbCameraConfig kDriverCamera =
      new UsbCameraConfig("DriverCam", 0, 320, 240, 12);

  // CPU-friendly vision processing rate. The vision thread sleeps between updates.
  public static final double kVisionProcessHz = 8.0;

  // Minimum decision margin to accept a detection for distance/pose updates.
  public static final double kMinDecisionMargin = 35.0;

  // AprilTag physical size in meters (default FRC tag size is 6.5 in).
  public static final double kAprilTagSizeMeters = 0.1651;

  // Approximate intrinsics for LifeCam HD-3000 at 320x240.
  // Replace with calibration values for best accuracy.
  public static final double kFxPixels = 240.0;
  public static final double kFyPixels = 240.0;
  public static final double kCxPixels = 160.0;
  public static final double kCyPixels = 120.0;

  // Robot-to-camera transform in robot coordinates.
  // Update for your real mount location and orientation.
  public static final Transform3d kRobotToAprilTagCamera =
      new Transform3d(new Translation3d(0.25, 0.0, 0.50), new Rotation3d());

  private CameraConstants() {}

  public record UsbCameraConfig(String name, int deviceIndex, int width, int height, int fps) {}
}
