// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// Swerve drivetrain subsystem with gyro-based odometry and PathPlanner integration.
public class DriveSubsystem extends SubsystemBase {
  private static final String kVisionRejectReasonKey = "Drive/VisionRejectReason";
  private static final String kVisionMaxLinearSpeedKey = "Drive/VisionMaxLinearSpeedMps";
  private static final String kVisionMaxAngularSpeedKey = "Drive/VisionMaxAngularSpeedDegPerSec";
  private static final String kVisionMaxPoseDeltaKey = "Drive/VisionMaxPoseDeltaMeters";
  private static final String kVisionMaxHeadingDeltaKey = "Drive/VisionMaxHeadingDeltaDeg";

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private Rotation2d getGyroRotation(){
    Rotation2d raw = m_gyro.getRotation2d();
    return DriveConstants.kGyroReversed
      ? Rotation2d.fromRadians(-raw.getRadians())
      : raw;
  }

  // Pose estimator blends module/gyro odometry with AprilTag vision updates.
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getGyroRotation(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2.5)),
      VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(12.0)));

  // Acceleration limiter: choose time to reach full linear/angular speed (seconds)
  private final DriveAccelerationLimiter m_accelLimiter = new DriveAccelerationLimiter(
      DriveConstants.kDriveTimeToMaxLinearSeconds,
      DriveConstants.kDriveTimeToMaxAngularSeconds);
  private double m_lastDriveTime = Double.NaN;
  // pathplanner; Configure robot from GUI settings
  RobotConfig config;
  private boolean m_useLowAutoPid = false;
  private boolean m_autoBuilderConfigured = false;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    try{

      // configure pathplanner autobuilder
      config = RobotConfig.fromGUISettings();
      configureAutoBuilder(
          new PIDConstants(AutoConstants.kPPTranslationP, AutoConstants.kPPTranslationI, AutoConstants.kPPTranslationD),
          new PIDConstants(AutoConstants.kPPRotationP, AutoConstants.kPPRotationI, AutoConstants.kPPRotationD));

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    SmartDashboard.putString(kVisionRejectReasonKey, "none");
    SmartDashboard.putNumber(kVisionMaxLinearSpeedKey, 2.5);
    SmartDashboard.putNumber(kVisionMaxAngularSpeedKey, 240.0);
    SmartDashboard.putNumber(kVisionMaxPoseDeltaKey, 1.5);
    SmartDashboard.putNumber(kVisionMaxHeadingDeltaKey, 35.0);
  }

  //pathplanner: get gyro heading
  public Rotation2d getGyroHeading() {
    return getGyroRotation();
  }

  public boolean isGyroConnected() {
    return m_gyro.isConnected();
  }

  // Reconfigure PathPlanner controller gains for auto-only tuning.
  public void setAutoPidMode(boolean useLowPid) {
    if (config == null || m_useLowAutoPid == useLowPid) {
      m_useLowAutoPid = useLowPid;
      return;
    }
    m_useLowAutoPid = useLowPid;
    DriverStation.reportWarning(
        "Auto PID mode toggle requested, but AutoBuilder is configured once at startup only.",
        false);
  }

  private void configureAutoBuilder(PIDConstants translation, PIDConstants rotation) {
    if (m_autoBuilderConfigured) {
      return;
    }

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            translation, // Translation PID constants
            rotation // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
    m_autoBuilderConfigured = true;
  }
  // resets robot pose
  public void resetPose(Pose2d pose) {
    // Reset your odometry / pose estimator to this pose.
    // Implementation depends on your odometry class / subsystem.
    m_poseEstimator.resetPosition(
          getGyroHeading(),
          new SwerveModulePosition[] {
             m_frontLeft.getPosition(),
             m_frontRight.getPosition(),
             m_rearLeft.getPosition(),
             m_rearRight.getPosition()
          },
          pose
        );
  }

  //pathplanner: get robot relative speeds
  public ChassisSpeeds getRobotRelativeSpeeds() {
    SwerveModuleState fl = m_frontLeft.getState();
    SwerveModuleState fr = m_frontRight.getState();
    SwerveModuleState bl = m_rearLeft.getState();
    SwerveModuleState br = m_rearRight.getState();
    ChassisSpeeds measuredSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(fl, fr, bl, br);
    return applyAutoTranslationFlip(measuredSpeeds);
  }
  
// pathplanner: drive robot relative
  public void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds correctedSpeeds = applyAutoTranslationFlip(speeds);
    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(correctedSpeeds, 0.02);
    // Convert the desired chassis velocity into individual module states
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(discretizedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

    // Apply the output to each module
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_rearLeft.setDesiredState(states[2]);
    m_rearRight.setDesiredState(states[3]);
  }
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_poseEstimator.update(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public boolean shouldAcceptVisionMeasurement(Pose2d visionPose) {
    if (visionPose == null) {
      SmartDashboard.putString(kVisionRejectReasonKey, "null-pose");
      return false;
    }

    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    double linearSpeedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double angularSpeedDegPerSec = Math.toDegrees(Math.abs(speeds.omegaRadiansPerSecond));
    double maxLinearSpeed = SmartDashboard.getNumber(kVisionMaxLinearSpeedKey, 2.5);
    double maxAngularSpeed = SmartDashboard.getNumber(kVisionMaxAngularSpeedKey, 240.0);
    if (linearSpeedMps > maxLinearSpeed) {
      SmartDashboard.putString(kVisionRejectReasonKey, "linear-speed");
      return false;
    }
    if (angularSpeedDegPerSec > maxAngularSpeed) {
      SmartDashboard.putString(kVisionRejectReasonKey, "angular-speed");
      return false;
    }

    Pose2d estimatedPose = getPose();
    double poseDeltaMeters =
        estimatedPose.getTranslation().getDistance(visionPose.getTranslation());
    double headingDeltaDeg =
        Math.abs(visionPose.getRotation().minus(estimatedPose.getRotation()).getDegrees());
    double maxPoseDelta = SmartDashboard.getNumber(kVisionMaxPoseDeltaKey, 1.5);
    double maxHeadingDelta = SmartDashboard.getNumber(kVisionMaxHeadingDeltaKey, 35.0);
    if (poseDeltaMeters > maxPoseDelta) {
      SmartDashboard.putString(kVisionRejectReasonKey, "pose-delta");
      return false;
    }
    if (headingDeltaDeg > maxHeadingDelta) {
      SmartDashboard.putString(kVisionRejectReasonKey, "heading-delta");
      return false;
    }

    SmartDashboard.putString(kVisionRejectReasonKey, "accepted");
    return true;
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // compute dt (seconds)
    double now = Timer.getFPGATimestamp();
    double dt = Double.isNaN(m_lastDriveTime) ? 0.02 : Math.max(1e-6, now - m_lastDriveTime);
    m_lastDriveTime = now;

    // Limit acceleration on normalized inputs (-1..1)
    double[] limited = m_accelLimiter.calculate(xSpeed, ySpeed, rot, dt);
    double limitedX = limited[0];
    double limitedY = limited[1];
    double limitedRot = limited[2];

    // Convert the commanded (limited) speeds into actual units for the drivetrain
    double xSpeedDelivered = limitedX * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = limitedY * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = limitedRot * DriveConstants.kMaxAngularSpeed;

    ChassisSpeeds commandedSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getGyroRotation())
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(commandedSpeeds, dt);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discretizedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
    // also reset limiter so there's no large jump when starting again
    m_accelLimiter.reset(0.0, 0.0, 0.0);
    m_lastDriveTime = Double.NaN;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getGyroRotation().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  private ChassisSpeeds applyAutoTranslationFlip(ChassisSpeeds speeds) {
    if (!DriveConstants.kAutoTranslationReversed) {
      return speeds;
    }

    return new ChassisSpeeds(
        -speeds.vxMetersPerSecond,
        -speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond);
  }
}
