// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.additionalSubSystems.intake;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.FuelLaunchConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subFuelLaunch.flyWheel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.vision.CameraServerWrapper;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final double kIntakeAxleMoveTimeoutSeconds = 1.5;
  private static final double kAimAtHubToleranceDeg = 1.5;
  private static final double kAimAtHubMaxTurnCommand = 0.35;
  private static final boolean kTeleopTranslationReversed = true;

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final intake m_intake = new intake();
  private final flyWheel m_shooter = new flyWheel(
      FuelLaunchConstants.kFlywheelCanId,
      FuelLaunchConstants.kShooterFeedCanId);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  //pathplanner: set up digital chooser for autos
  private final SendableChooser<Command> autoChooser;
  private boolean m_fieldRelativeEnabled = true;
  private boolean m_xPressLowersIntake = true;
  private boolean m_intakeFeedRunning = false;
  private double m_intakeFeedPercent = 0.0;
  private final CameraServerWrapper m_cameraServerWrapper = new CameraServerWrapper();
  private static final String kShooterSpeedPercentKey = "Shooter/TestSpeedPercent";
  private GenericEntry m_driverControlsEntry;
  private GenericEntry m_shooterActiveEntry;
  private GenericEntry m_shooterSpeedPercentEntry;
  private GenericEntry m_shooterTargetRpmEntry;
  private GenericEntry m_shooterActualRpmEntry;
  private GenericEntry m_shooterFeedActualRpmEntry;
  private GenericEntry m_intakeForwardPressCountEntry;
  private GenericEntry m_intakeReversePressCountEntry;
  private GenericEntry m_shooterStartPressCountEntry;
  private GenericEntry m_shooterStopPressCountEntry;
  private int m_intakeForwardPressCount = 0;
  private int m_intakeReversePressCount = 0;
  private int m_shooterStartPressCount = 0;
  private int m_shooterStopPressCount = 0;
  private final PIDController m_aimAtHubController = new PIDController(0.02, 0.0, 0.001);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Start camera processing before any commands read vision distance/yaw.
    m_cameraServerWrapper.initialize();
    m_aimAtHubController.setTolerance(kAimAtHubToleranceDeg);

    // Bring mechanisms to a known stopped state on boot.
    initializeIntakeOnBoot();
    initializeShooterOnBoot();
    // Register pathplanner/autonomous hooks once subsystems are ready.
    registerNamedCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            this::driveFromController,
            m_robotDrive));
    // pathplanner: build chooser with only competition autos
    autoChooser = buildCompetitionAutoChooser();
    SmartDashboard.putNumber(kShooterSpeedPercentKey, 1);
    configureDriverDashboard();
  }

  private void initializeIntakeOnBoot() {
    stopIntake();
  }

  private void setIntakeFeedRunning(boolean running, double percent) {
    m_intakeFeedRunning = running;
    m_intakeFeedPercent = running ? percent : 0.0;
    if (running) {
      m_intake.setFeedPercent(m_intakeFeedPercent);
    } else {
      m_intake.stopFeed();
    }
    updateDriverControlsDashboard();
  }

  private void toggleIntakeFeed(double percent) {
    boolean sameDirectionRequested = m_intakeFeedRunning && Math.signum(m_intakeFeedPercent) == Math.signum(percent);
    setIntakeFeedRunning(!sameDirectionRequested, percent);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "lower",
        lowerIntakeCommand());

    NamedCommands.registerCommand(
        "up",
        raiseIntakeCommand());

    NamedCommands.registerCommand(
        "start spin",
        m_intake.feedPercentCommand(-1));

    NamedCommands.registerCommand(
        "stop spin",
        new InstantCommand(m_intake::stopFeed, m_intake));

    NamedCommands.registerCommand(
        "shoot",
        // Active path during tuning: uses the dashboard percent entry.
        shootFromVisionDistanceCommand());

    NamedCommands.registerCommand(
        "stop shoot",
        new InstantCommand(m_shooter::stopAll, m_shooter));

    NamedCommands.registerCommand(
        "clear feed",
        new InstantCommand(() -> m_shooter.reverseFeedWheel(1.0), m_shooter));
  }

  private Command lowerIntakeCommand() {
    // Run until the lower limit is hit, then stop the axle motor cleanly.
    return m_intake.axlePercentCommand(-0.25)
        .until(m_intake::isReverseLimitPressed)
        .withTimeout(kIntakeAxleMoveTimeoutSeconds)
        .andThen(new InstantCommand(m_intake::stopAxle, m_intake));
  }

  private Command raiseIntakeCommand() {
    // Mirror of lowerIntakeCommand(), but toward the stowed limit switch.
    return m_intake.axlePercentCommand(0.25)
        .until(m_intake::isForwardLimitPressed)
        .withTimeout(kIntakeAxleMoveTimeoutSeconds)
        .andThen(new InstantCommand(m_intake::stopAxle, m_intake));
  }

  // Active shooter command path while the lookup table is still being tuned.
  // When the table is ready, swap callers over to shootFromVisionDistanceCommand().
  private Command shootFromDashboardSpeedCommand() {
    return new InstantCommand(() -> {
      // Driver-selected percent is useful while shooter table values are still being tuned.
      double flywheelSpeedPercent = getShooterSpeedPercent();
      m_shooter.startShootingAtPercent(flywheelSpeedPercent);
    }, m_shooter);
  }

  // Ready-to-use lookup-table path once tuning is complete.
  // This reads the camera's current target distance, looks up flywheel/feed values,
  // and falls back to the dashboard speed if no target is available.
  private Command shootFromVisionDistanceCommand() {
    return new InstantCommand(() -> {
      if (m_cameraServerWrapper.hasTarget()) {
        // Vision distance feeds directly into the shooter lookup table.
        double distanceMeters = m_cameraServerWrapper.getDistanceMeters();
        m_shooter.startShootingForDistanceMeters(distanceMeters);
      } else {
        // Fallback keeps testing possible if vision drops out.
        double flywheelSpeedPercent = getShooterSpeedPercent();
        m_shooter.startShootingAtPercent(flywheelSpeedPercent);
      }
    }, m_shooter);
  }

  private void initializeShooterOnBoot() {
    m_shooter.stopAll();
  }
  //Right trigger scales translation + rotation speed down for precission
  private double getSpeedScale(){
    double trigger = MathUtil.applyDeadband(m_driverController.getRightTriggerAxis(),0.05);
    double minScale = MathUtil.clamp(OIConstants.kTriggerSlowMinScale, 0.0, 1.0);
    return 1.0-trigger*(1.0-minScale);
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            // Point modules inward to resist being pushed around.
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
            // Driver keeps control of translation while heading is auto-corrected from vision yaw.
            this::driveWhileAimingAtHub,
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
        .onTrue(new InstantCommand(() -> {
          m_shooterStopPressCount++;
          m_shooter.stopAll();
          updateDriverControlsDashboard();
        }, m_shooter));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new InstantCommand(() -> {
          if (m_xPressLowersIntake) {
            lowerIntakeCommand().schedule();
          } else {
            raiseIntakeCommand().schedule();
          }
          m_xPressLowersIntake = !m_xPressLowersIntake;
          updateDriverControlsDashboard();
        }));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> {
          m_intakeForwardPressCount++;
          toggleIntakeFeed(-1.0);
          updateDriverControlsDashboard();
        }, m_intake));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(() -> {
          m_intakeReversePressCount++;
          toggleIntakeFeed(1.0);
          updateDriverControlsDashboard();
        }, m_intake));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> {
          m_shooterStartPressCount++;
          shootFromVisionDistanceCommand().schedule();
          updateDriverControlsDashboard();
        }, m_shooter));

    new Trigger(() -> m_driverController.getPOV() == 180)
        .onTrue(new InstantCommand(
            () -> {
              m_fieldRelativeEnabled = !m_fieldRelativeEnabled;
              updateDriverControlsDashboard();
            }));

    new Trigger(() -> m_driverController.getPOV() == 0)
        .onTrue(new InstantCommand(m_cameraServerWrapper::toggleReadEnabled));

  }

  // when running a command for autonomous, call this to get the command
  public Command getAutonomousCommand() {
    // PathPlanner chooser already contains the competition-only autos.
    return autoChooser.getSelected();
  }

  private SendableChooser<Command> buildCompetitionAutoChooser() {
    SendableChooser<Command> chooser = new SendableChooser<>();
    List<String> autoNames = getCompetitionAutoNames();

    if (autoNames.isEmpty()) {
      chooser.setDefaultOption("No Comp Autos Found", null);
      return chooser;
    }

    boolean defaultSet = false;
    for (String autoName : autoNames) {
      Command autoCommand = new PathPlannerAuto(autoName);
      if (!defaultSet) {
        chooser.setDefaultOption(autoName, autoCommand);
        defaultSet = true;
      } else {
        chooser.addOption(autoName, autoCommand);
      }
    }

    return chooser;
  }

  private List<String> getCompetitionAutoNames() {
    Path autosDirectory = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos");
    if (!Files.isDirectory(autosDirectory)) {
      return List.of();
    }

    try (Stream<Path> autoFiles = Files.list(autosDirectory)) {
      return autoFiles
          .map(path -> path.getFileName().toString())
          .filter(fileName -> fileName.endsWith(".auto"))
          .map(fileName -> fileName.substring(0, fileName.length() - ".auto".length()))
          .filter(autoName -> autoName.startsWith("Comp-"))
          .sorted()
          .toList();
    } catch (IOException exception) {
      return List.of();
    }
  }

  public void setAutoPidMode(boolean useLowPid){
    m_robotDrive.setAutoPidMode(useLowPid);
  }

  public void periodic() {
    // Vision still updates dashboard/shooting values even though drivetrain pose fusion is disabled.
    m_cameraServerWrapper.periodic();
    updateDriverControlsDashboard();
  }

  private void driveFromController() {
    double xInput = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * getSpeedScale();
    double yInput = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * getSpeedScale();
    double rotInput = -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband) * getSpeedScale();

    if (kTeleopTranslationReversed) {
      xInput = -xInput;
      yInput = -yInput;
    }

    m_robotDrive.drive(xInput, yInput, rotInput, m_fieldRelativeEnabled);
  }

  private void driveWhileAimingAtHub() {
    double xSpeed = -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * getSpeedScale();
    double ySpeed = -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * getSpeedScale();
    double rotSpeed = 0.0;

    if (m_cameraServerWrapper.hasTarget()) {
      // Positive yaw means the target is off-center; PID drives that error back to zero.
      double yawErrorDeg = m_cameraServerWrapper.getYawDegrees();
      rotSpeed = MathUtil.clamp(
          -m_aimAtHubController.calculate(yawErrorDeg, 0.0),
          -kAimAtHubMaxTurnCommand,
          kAimAtHubMaxTurnCommand);

      if (m_aimAtHubController.atSetpoint()) {
        rotSpeed = 0.0;
      }
    } else {
      m_aimAtHubController.reset();
    }

    m_robotDrive.drive(xSpeed, ySpeed, rotSpeed, m_fieldRelativeEnabled);
  }

  public void stopIntake() {
    m_intake.stopAll();
    m_intakeFeedRunning = false;
  }

  private void configureDriverDashboard() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    m_driverControlsEntry = driverTab.add("Driver Controls", "").withSize(6, 4).getEntry();
    m_shooterActiveEntry = driverTab.add("Shooter Active", false).withSize(2, 1).getEntry();
    m_shooterSpeedPercentEntry = driverTab.add("Flywheel Speed %", 0.75).withSize(2, 1).getEntry();
    m_shooterTargetRpmEntry = driverTab.add("Shooter Target RPM", 0).withSize(2, 1).getEntry();
    m_shooterActualRpmEntry = driverTab.add("Shooter Actual RPM", 0).withSize(2, 1).getEntry();
    m_shooterFeedActualRpmEntry = driverTab.add("Feed Actual RPM", 0).withSize(2, 1).getEntry();
    m_intakeForwardPressCountEntry = driverTab.add("Intake Y Presses", 0).withSize(2, 1).getEntry();
    m_intakeReversePressCountEntry = driverTab.add("Intake B Presses", 0).withSize(2, 1).getEntry();
    m_shooterStartPressCountEntry = driverTab.add("Shooter A Presses", 0).withSize(2, 1).getEntry();
    m_shooterStopPressCountEntry = driverTab.add("Shooter Stop Presses", 0).withSize(2, 1).getEntry();
    driverTab.addBoolean("Gyro Connected", m_robotDrive::isGyroConnected).withSize(2, 1);
    driverTab.addBoolean("Cameras Working", m_cameraServerWrapper::areCamerasWorking).withSize(2, 1);
    driverTab.addBoolean("Vision Working", m_cameraServerWrapper::hasTarget).withSize(2, 1);
    driverTab.addDouble("Target Distance (m)", m_cameraServerWrapper::getDistanceMeters).withSize(2, 1);
    driverTab.addBoolean("Field Relative", () -> m_fieldRelativeEnabled).withSize(2, 1);
    driverTab.add("Auto Chooser", autoChooser).withSize(4, 2);
    Shuffleboard.selectTab("Driver");
    updateDriverControlsDashboard();
  }

  private void updateDriverControlsDashboard() {
    if (m_driverControlsEntry == null) {
      return;
    }
    m_driverControlsEntry.setString(buildDriverControlsSummary());
    if (m_shooterActiveEntry != null) {
      m_shooterActiveEntry.setBoolean(m_shooter.isShooting());
    }
    if (m_shooterTargetRpmEntry != null) {
      m_shooterTargetRpmEntry.setDouble(m_shooter.getTargetFlywheelRPM());
    }
    if (m_shooterActualRpmEntry != null) {
      m_shooterActualRpmEntry.setDouble(m_shooter.getFlyWheelSpeedRPM());
    }
    if (m_shooterFeedActualRpmEntry != null) {
      m_shooterFeedActualRpmEntry.setDouble(m_shooter.getFeedSpeedRPM());
    }
    if (m_intakeForwardPressCountEntry != null) {
      m_intakeForwardPressCountEntry.setDouble(m_intakeForwardPressCount);
    }
    if (m_intakeReversePressCountEntry != null) {
      m_intakeReversePressCountEntry.setDouble(m_intakeReversePressCount);
    }
    if (m_shooterStartPressCountEntry != null) {
      m_shooterStartPressCountEntry.setDouble(m_shooterStartPressCount);
    }
    if (m_shooterStopPressCountEntry != null) {
      m_shooterStopPressCountEntry.setDouble(m_shooterStopPressCount);
    }
  }

  private double getShooterSpeedPercent() {
    // Shuffleboard entry wins if present; SmartDashboard value is kept in sync as a fallback.
    /*
    double dashboardPercent = SmartDashboard.getNumber(kShooterSpeedPercentKey, 0.75);
    if (m_shooterSpeedPercentEntry == null) {
      return MathUtil.clamp(dashboardPercent, 0.0, 1.0);
    }
    double tabPercent = MathUtil.clamp(m_shooterSpeedPercentEntry.getDouble(dashboardPercent), 0.0, 1.0);
    SmartDashboard.putNumber(kShooterSpeedPercentKey, tabPercent);
     */
    double tabPercent= 0.60;
    return tabPercent;
  }

  private String buildDriverControlsSummary() {
    String xAction = m_xPressLowersIntake ? "Lower intake" : "Raise intake";
    String intakeFeedState = m_intakeFeedRunning ? "ON" : "OFF";
    String intakeFeedDirection = m_intakeFeedPercent > 0.0 ? "Reverse" : "Forward";
    String shooterState = m_shooter.isShooting() ? "ON" : "OFF";
    String fieldRelativeState = m_fieldRelativeEnabled ? "ON" : "OFF";
    String visionReadState = m_cameraServerWrapper.isReadEnabled() ? "ON" : "OFF";

    return String.join(
        "\n",
        formatControlLine("X", xAction, m_driverController.getXButton()),
        formatControlLine("Y", "Intake forward " + intakeFeedState, m_driverController.getYButton()),
        formatControlLine("B", "Intake reverse " + intakeFeedState, m_driverController.getBButton()),
        formatControlLine("A", "Start shooter (shooter " + shooterState + ", mode Vision LUT)", m_driverController.getAButton()),
        formatControlLine("L1", "Aim at hub", m_driverController.getLeftBumperButton()),
        formatControlLine("R1", "Swerve X (brake)", m_driverController.getRightBumperButton()),
        formatControlLine("Back", "Stop shooter", m_driverController.getBackButton()),
        formatControlLine("Start", "Zero heading", m_driverController.getStartButton()),
        formatControlLine("POV Up", "Toggle vision read (vision " + visionReadState + ")", m_driverController.getPOV() == 0),
        formatControlLine("POV Down", "Field relative " + fieldRelativeState, m_driverController.getPOV() == 180));
  }

  private String formatControlLine(String button, String action, boolean pressed) {
    return String.format("%s: %s [%s]", button, action, pressed ? "PRESSED" : "idle");
  }
  /*

// default command for auto: however, we want to call pathplanner autos instead
  
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }*/
}
