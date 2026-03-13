package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Comparator;
import java.util.Optional;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class CameraServerWrapper {
  private static final String kReadEnabledKey = "Vision/ReadEnabled";
  private static final String kConnectedCountKey = "Vision/ConnectedCameraCount";
  private static final String kHasTargetKey = "Vision/AprilTag/HasTarget";
  private static final String kBestIdKey = "Vision/AprilTag/BestId";
  private static final String kDistanceMetersKey = "Vision/AprilTag/DistanceMeters";
  private static final String kDistanceFeetKey = "Vision/AprilTag/DistanceFeet";
  private static final String kLineDistanceMetersKey = "Vision/AprilTag/LineDistanceMeters";
  private static final String kYawDegreesKey = "Vision/AprilTag/YawDegrees";
  private static final String kPoseXKey = "Vision/AprilTag/PoseX";
  private static final String kPoseYKey = "Vision/AprilTag/PoseY";
  private static final String kPoseZKey = "Vision/AprilTag/PoseZ";
  private static final String kVisionSummaryKey = "Vision/Summary";
  private static final String kVisionPublishAdvancedKey = "Vision/PublishAdvanced";
  private static final String kProcessHzKey = "Vision/ProcessHz";
  private static final String kFieldPoseValidKey = "Vision/RobotPoseValid";
  private static final String kFieldPoseXKey = "Vision/RobotPose2dX";
  private static final String kFieldPoseYKey = "Vision/RobotPose2dY";
  private static final String kFieldPoseHeadingDegKey = "Vision/RobotPose2dHeadingDeg";
  private static final String kDecisionMarginKey = "Vision/AprilTag/DecisionMargin";
  private static final String kMinDecisionMarginKey = "Vision/AprilTag/MinDecisionMargin";
  private static final String kCalibrationEnabledKey = "Vision/Calibration/Enabled";
  private static final String kExpectedDistanceMetersKey = "Vision/Calibration/ExpectedDistanceMeters";
  private static final String kDistanceErrorMetersKey = "Vision/Calibration/DistanceErrorMeters";
  private static final String kExpectedPoseXKey = "Vision/Calibration/ExpectedRobotPoseX";
  private static final String kExpectedPoseYKey = "Vision/Calibration/ExpectedRobotPoseY";
  private static final String kExpectedHeadingDegKey = "Vision/Calibration/ExpectedRobotHeadingDeg";
  private static final String kPoseErrorXMetersKey = "Vision/Calibration/PoseErrorX";
  private static final String kPoseErrorYMetersKey = "Vision/Calibration/PoseErrorY";
  private static final String kPoseErrorMetersKey = "Vision/Calibration/PoseErrorNorm";
  private static final String kHeadingErrorDegKey = "Vision/Calibration/HeadingErrorDeg";

  private UsbCamera aprilTagCamera;
  private UsbCamera driverCamera;
  private CvSink aprilTagSink;
  private AprilTagDetector detector;
  private AprilTagPoseEstimator poseEstimator;
  private AprilTagFieldLayout fieldLayout;
  private Thread processingThread;

  private boolean readEnabled = CameraConstants.kReadEnabledByDefault;
  private volatile boolean processingThreadRunning = false;
  private volatile int connectedCameraCount = 0;
  private volatile boolean hasTarget = false;
  private volatile double floorDistanceMeters = 0.0;
  private volatile double lineDistanceMeters = 0.0;
  private volatile double yawDegrees = 0.0;
  private volatile VisionMeasurement latestVisionMeasurement;

  public static record VisionMeasurement(
      Pose2d pose,
      double timestampSeconds,
      double floorDistanceMeters,
      double lineDistanceMeters,
      double decisionMargin,
      int tagId) {}

  public void initialize() {
    SmartDashboard.putBoolean(kReadEnabledKey, readEnabled);
    SmartDashboard.putBoolean(kHasTargetKey, false);
    SmartDashboard.putNumber(kBestIdKey, -1);
    SmartDashboard.putNumber(kDistanceMetersKey, 0.0);
    SmartDashboard.putNumber(kDistanceFeetKey, 0.0);
    SmartDashboard.putNumber(kLineDistanceMetersKey, 0.0);
    SmartDashboard.putString(kVisionSummaryKey, "target:none");
    SmartDashboard.putBoolean(kVisionPublishAdvancedKey, false);
    SmartDashboard.putNumber(kProcessHzKey, CameraConstants.kVisionProcessHz);
    SmartDashboard.putBoolean(kFieldPoseValidKey, false);
    SmartDashboard.putNumber(kFieldPoseXKey, 0.0);
    SmartDashboard.putNumber(kFieldPoseYKey, 0.0);
    SmartDashboard.putNumber(kFieldPoseHeadingDegKey, 0.0);
    SmartDashboard.putNumber(kMinDecisionMarginKey, CameraConstants.kMinDecisionMargin);
    SmartDashboard.putBoolean(kCalibrationEnabledKey, false);

    int connectedCount = 0;
    connectedCount += initializeAprilTagCamera() ? 1 : 0;
    connectedCount += initializeDriverCamera() ? 1 : 0;
    connectedCameraCount = connectedCount;
    SmartDashboard.putNumber(kConnectedCountKey, connectedCount);

    detector = new AprilTagDetector();
    detector.addFamily("tag36h11", 1);
    poseEstimator =
        new AprilTagPoseEstimator(
            new AprilTagPoseEstimator.Config(
                CameraConstants.kAprilTagSizeMeters,
                CameraConstants.kFxPixels,
                CameraConstants.kFyPixels,
                CameraConstants.kCxPixels,
                CameraConstants.kCyPixels));
    initializeFieldLayout();

    if (aprilTagSink != null) {
      startProcessingThread();
    }
  }

  private void initializeFieldLayout() {
    try {
      fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    } catch (Exception ex) {
      SmartDashboard.putString("Vision/Error/FieldLayout", "Failed to load default field AprilTag layout");
    }
  }

  private boolean initializeAprilTagCamera() {
    CameraConstants.UsbCameraConfig cameraConfig = CameraConstants.kAprilTagCamera;
    try {
      aprilTagCamera = CameraServer.startAutomaticCapture(cameraConfig.name(), cameraConfig.deviceIndex());
      aprilTagCamera.setResolution(cameraConfig.width(), cameraConfig.height());
      aprilTagCamera.setFPS(cameraConfig.fps());
      aprilTagCamera.setConnectionStrategy(readEnabled ? ConnectionStrategy.kKeepOpen : ConnectionStrategy.kForceClose);
      aprilTagSink = CameraServer.getVideo(aprilTagCamera);
      return true;
    } catch (VideoException ex) {
      SmartDashboard.putString(
          "Vision/Error/" + cameraConfig.name(),
          "Camera index " + cameraConfig.deviceIndex() + " not available");
      return false;
    }
  }

  private boolean initializeDriverCamera() {
    CameraConstants.UsbCameraConfig cameraConfig = CameraConstants.kDriverCamera;
    try {
      driverCamera = CameraServer.startAutomaticCapture(cameraConfig.name(), cameraConfig.deviceIndex());
      driverCamera.setResolution(cameraConfig.width(), cameraConfig.height());
      driverCamera.setFPS(cameraConfig.fps());
      driverCamera.setConnectionStrategy(readEnabled ? ConnectionStrategy.kKeepOpen : ConnectionStrategy.kForceClose);
      return true;
    } catch (VideoException ex) {
      SmartDashboard.putString(
          "Vision/Error/" + cameraConfig.name(),
          "Camera index " + cameraConfig.deviceIndex() + " not available");
      return false;
    }
  }

  private void startProcessingThread() {
    processingThreadRunning = true;
    processingThread =
        new Thread(
            () -> {
              Mat frame = new Mat();
              Mat gray = new Mat();
              long lastCycleStartNanos = System.nanoTime();

              while (processingThreadRunning) {
                if (!readEnabled || aprilTagSink == null) {
                  sleepSeconds(0.02);
                  lastCycleStartNanos = System.nanoTime();
                  continue;
                }

                double processHz =
                    Math.max(1.0, SmartDashboard.getNumber(kProcessHzKey, CameraConstants.kVisionProcessHz));
                long cyclePeriodNanos = (long) (1_000_000_000.0 / processHz);
                long nowNanos = System.nanoTime();
                long elapsedNanos = nowNanos - lastCycleStartNanos;
                if (elapsedNanos < cyclePeriodNanos) {
                  sleepSeconds((cyclePeriodNanos - elapsedNanos) / 1_000_000_000.0);
                }
                lastCycleStartNanos = System.nanoTime();

                long frameTime = aprilTagSink.grabFrame(frame);
                if (frameTime == 0 || frame.empty()) {
                  SmartDashboard.putBoolean(kHasTargetKey, false);
                  hasTarget = false;
                  floorDistanceMeters = 0.0;
                  lineDistanceMeters = 0.0;
                  yawDegrees = 0.0;
                  latestVisionMeasurement = null;
                  SmartDashboard.putBoolean(kFieldPoseValidKey, false);
                  SmartDashboard.putString(kVisionSummaryKey, "target:none");
                  sleepSeconds(0.01);
                  continue;
                }

                Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);
                AprilTagDetection[] detections = detector.detect(gray);
                publishBestDetection(detections);
              }

              frame.release();
              gray.release();
            });

    processingThread.setName("AprilTagProcessingThread");
    processingThread.setDaemon(true);
    processingThread.start();
  }

  private void publishBestDetection(AprilTagDetection[] detections) {
    if (detections.length == 0) {
      SmartDashboard.putBoolean(kHasTargetKey, false);
      hasTarget = false;
      floorDistanceMeters = 0.0;
      lineDistanceMeters = 0.0;
      yawDegrees = 0.0;
      latestVisionMeasurement = null;
      SmartDashboard.putBoolean(kFieldPoseValidKey, false);
      SmartDashboard.putString(kVisionSummaryKey, "target:none");
      return;
    }

    AprilTagDetection bestDetection =
        java.util.Arrays.stream(detections)
            .max(Comparator.comparingDouble(AprilTagDetection::getDecisionMargin))
            .orElse(detections[0]);
    double decisionMargin = bestDetection.getDecisionMargin();
    double minDecisionMargin =
        SmartDashboard.getNumber(kMinDecisionMarginKey, CameraConstants.kMinDecisionMargin);
    boolean publishAdvanced = SmartDashboard.getBoolean(kVisionPublishAdvancedKey, false);
    if (publishAdvanced) {
      SmartDashboard.putNumber(kDecisionMarginKey, decisionMargin);
    }
    if (decisionMargin < minDecisionMargin) {
      SmartDashboard.putBoolean(kHasTargetKey, false);
      hasTarget = false;
      floorDistanceMeters = 0.0;
      lineDistanceMeters = 0.0;
      yawDegrees = 0.0;
      latestVisionMeasurement = null;
      SmartDashboard.putBoolean(kFieldPoseValidKey, false);
      SmartDashboard.putString(
          kVisionSummaryKey,
          String.format("target:reject id:%d margin:%.1f", bestDetection.getId(), decisionMargin));
      return;
    }

    Transform3d cameraToTag = poseEstimator.estimate(bestDetection);
    if (cameraToTag == null) {
      SmartDashboard.putBoolean(kHasTargetKey, false);
      hasTarget = false;
      floorDistanceMeters = 0.0;
      lineDistanceMeters = 0.0;
      yawDegrees = 0.0;
      latestVisionMeasurement = null;
      SmartDashboard.putBoolean(kFieldPoseValidKey, false);
      SmartDashboard.putString(kVisionSummaryKey, "target:bad-pose");
      return;
    }

    double xMeters = cameraToTag.getX();
    double yMeters = cameraToTag.getY();
    double zMeters = cameraToTag.getZ();
    double floorDistanceMeters = Math.hypot(xMeters, yMeters);
    double lineDistanceMeters = Math.sqrt(xMeters * xMeters + yMeters * yMeters + zMeters * zMeters);
    double yawDegrees = Math.toDegrees(Math.atan2(yMeters, xMeters));
    Optional<Pose2d> fieldToRobot2d = estimateFieldRelativeRobotPose(bestDetection, cameraToTag);

    hasTarget = true;
    this.floorDistanceMeters = floorDistanceMeters;
    this.lineDistanceMeters = lineDistanceMeters;
    this.yawDegrees = yawDegrees;
    SmartDashboard.putBoolean(kHasTargetKey, true);
    SmartDashboard.putNumber(kBestIdKey, bestDetection.getId());
    SmartDashboard.putNumber(kDistanceMetersKey, floorDistanceMeters);
    SmartDashboard.putNumber(kDistanceFeetKey, Units.metersToFeet(floorDistanceMeters));
    SmartDashboard.putNumber(kLineDistanceMetersKey, lineDistanceMeters);
    if (publishAdvanced) {
      SmartDashboard.putNumber(kYawDegreesKey, yawDegrees);
      SmartDashboard.putNumber(kPoseXKey, xMeters);
      SmartDashboard.putNumber(kPoseYKey, yMeters);
      SmartDashboard.putNumber(kPoseZKey, zMeters);
    }
    publishDistanceCalibrationError(floorDistanceMeters);
    if (fieldToRobot2d.isPresent()) {
      Pose2d robotPose = fieldToRobot2d.get();
      latestVisionMeasurement =
          new VisionMeasurement(
              robotPose,
              Timer.getFPGATimestamp(),
              floorDistanceMeters,
              lineDistanceMeters,
              decisionMargin,
              bestDetection.getId());
      publishFieldRelativeRobotPose(robotPose);
    } else {
      latestVisionMeasurement = null;
      SmartDashboard.putBoolean(kFieldPoseValidKey, false);
    }
    SmartDashboard.putString(
        kVisionSummaryKey,
        String.format(
            "id:%d d:%.2fm yaw:%.1f pose:%s",
            bestDetection.getId(),
            floorDistanceMeters,
            yawDegrees,
            SmartDashboard.getBoolean(kFieldPoseValidKey, false) ? "ok" : "no"));
  }

  private Optional<Pose2d> estimateFieldRelativeRobotPose(
      AprilTagDetection bestDetection, Transform3d cameraToTag) {
    if (fieldLayout == null) {
      return Optional.empty();
    }

    Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(bestDetection.getId());
    if (tagPoseOpt.isEmpty()) {
      return Optional.empty();
    }

    Pose3d fieldToTag = tagPoseOpt.get();
    Pose3d fieldToCamera = fieldToTag.transformBy(cameraToTag.inverse());
    Pose3d fieldToRobot = fieldToCamera.transformBy(CameraConstants.kRobotToAprilTagCamera.inverse());
    return Optional.of(fieldToRobot.toPose2d());
  }

  private void publishFieldRelativeRobotPose(Pose2d fieldToRobot2d) {
    SmartDashboard.putBoolean(kFieldPoseValidKey, true);
    SmartDashboard.putNumber(kFieldPoseXKey, fieldToRobot2d.getX());
    SmartDashboard.putNumber(kFieldPoseYKey, fieldToRobot2d.getY());
    SmartDashboard.putNumber(kFieldPoseHeadingDegKey, fieldToRobot2d.getRotation().getDegrees());
    publishPoseCalibrationError(fieldToRobot2d);
  }

  public void periodic() {
    boolean desiredReadEnabled = SmartDashboard.getBoolean(kReadEnabledKey, readEnabled);
    if (desiredReadEnabled != readEnabled) {
      setReadEnabled(desiredReadEnabled);
    }
  }

  public void toggleReadEnabled() {
    setReadEnabled(!readEnabled);
  }

  public void setReadEnabled(boolean enabled) {
    readEnabled = enabled;
    SmartDashboard.putBoolean(kReadEnabledKey, readEnabled);

    if (aprilTagCamera != null) {
      aprilTagCamera.setConnectionStrategy(readEnabled ? ConnectionStrategy.kKeepOpen : ConnectionStrategy.kForceClose);
    }
    if (driverCamera != null) {
      driverCamera.setConnectionStrategy(readEnabled ? ConnectionStrategy.kKeepOpen : ConnectionStrategy.kForceClose);
    }
  }

  public boolean isReadEnabled() {
    return readEnabled;
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public boolean areCamerasWorking() {
    return connectedCameraCount > 0;
  }

  public int getConnectedCameraCount() {
    return connectedCameraCount;
  }

  public double getDistanceMeters() {
    return floorDistanceMeters;
  }

  public double getLineDistanceMeters() {
    return lineDistanceMeters;
  }

  public double getYawDegrees() {
    return yawDegrees;
  }

  public Optional<VisionMeasurement> getLatestVisionMeasurement() {
    return Optional.ofNullable(latestVisionMeasurement);
  }

  public void close() {
    processingThreadRunning = false;
    if (processingThread != null) {
      processingThread.interrupt();
    }
    if (detector != null) {
      detector.close();
    }
  }

  private void sleepSeconds(double seconds) {
    try {
      Thread.sleep((long) (seconds * 1000.0));
    } catch (InterruptedException ignored) {
      Thread.currentThread().interrupt();
    }
  }

  private void publishDistanceCalibrationError(double measuredDistanceMeters) {
    if (!SmartDashboard.getBoolean(kCalibrationEnabledKey, false)) {
      return;
    }

    double expectedDistanceMeters = SmartDashboard.getNumber(kExpectedDistanceMetersKey, 0.0);
    double distanceErrorMeters = measuredDistanceMeters - expectedDistanceMeters;
    SmartDashboard.putNumber(kDistanceErrorMetersKey, distanceErrorMeters);
  }

  private void publishPoseCalibrationError(Pose2d measuredRobotPose) {
    if (!SmartDashboard.getBoolean(kCalibrationEnabledKey, false)) {
      return;
    }

    double expectedPoseX = SmartDashboard.getNumber(kExpectedPoseXKey, 0.0);
    double expectedPoseY = SmartDashboard.getNumber(kExpectedPoseYKey, 0.0);
    double expectedHeadingDeg = SmartDashboard.getNumber(kExpectedHeadingDegKey, 0.0);

    double poseErrorX = measuredRobotPose.getX() - expectedPoseX;
    double poseErrorY = measuredRobotPose.getY() - expectedPoseY;
    double poseErrorMeters = Math.hypot(poseErrorX, poseErrorY);
    double headingErrorDeg =
        wrapDegrees(measuredRobotPose.getRotation().getDegrees() - expectedHeadingDeg);

    SmartDashboard.putNumber(kPoseErrorXMetersKey, poseErrorX);
    SmartDashboard.putNumber(kPoseErrorYMetersKey, poseErrorY);
    SmartDashboard.putNumber(kPoseErrorMetersKey, poseErrorMeters);
    SmartDashboard.putNumber(kHeadingErrorDegKey, headingErrorDeg);
  }

  private double wrapDegrees(double angleDeg) {
    double wrapped = angleDeg % 360.0;
    if (wrapped > 180.0) {
      wrapped -= 360.0;
    } else if (wrapped < -180.0) {
      wrapped += 360.0;
    }
    return wrapped;
  }
}
