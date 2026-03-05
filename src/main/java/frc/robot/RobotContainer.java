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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.additionalSubSystems.intake;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
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
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.vision.CameraServerWrapper;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
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
  private boolean m_aPressStartsShooter = true;
  private final CameraServerWrapper m_cameraServerWrapper = new CameraServerWrapper();
  private static final String kShooterDistanceMetersKey = "Shooter/TestDistanceMeters";

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_cameraServerWrapper.initialize();

    initializeIntakeOnBoot();
    initializeShooterOnBoot();
    registerNamedCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)*getSpeedScale(),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)*getSpeedScale(),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)*getSpeedScale(),
                m_fieldRelativeEnabled),
            m_robotDrive));
            // pathplanner: build auto chooser and put on dashboard
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putBoolean("Drive Field Relative Enabled", m_fieldRelativeEnabled);
    SmartDashboard.putNumber(kShooterDistanceMetersKey, 2.0);
    
  }

  private void initializeIntakeOnBoot() {
    m_intake.stopFeed();
    m_intake.setAxlePositionRotations(IntakeConstants.kIntakeAxleMaxRotations);
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
        m_intake.feedPercentCommand(0.5));

    NamedCommands.registerCommand(
        "stop spin",
        new InstantCommand(m_intake::stopFeed, m_intake));

    NamedCommands.registerCommand(
        "shoot",
        shootFromDashboardDistanceCommand());

    NamedCommands.registerCommand(
        "stop shoot",
        new InstantCommand(m_shooter::stopAll, m_shooter));

    NamedCommands.registerCommand(
        "clear feed",
        new InstantCommand(() -> m_shooter.reverseFeedWheel(1.0), m_shooter));
  }

  private Command lowerIntakeCommand() {
    return m_intake.axlePercentCommand(-0.25)
        .until(m_intake::isReverseLimitPressed)
        .andThen(new InstantCommand(m_intake::stopAxle, m_intake));
  }

  private Command raiseIntakeCommand() {
    return m_intake.axlePercentCommand(0.25)
        .until(m_intake::isForwardLimitPressed)
        .andThen(new InstantCommand(m_intake::stopAxle, m_intake));
  }

  private Command shootFromDashboardDistanceCommand() {
    return new InstantCommand(() -> {
      double distanceMeters = SmartDashboard.getNumber(kShooterDistanceMetersKey, 2.0);
      m_shooter.startShootingForDistanceMeters(distanceMeters);
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
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new InstantCommand(() -> {
          if (m_xPressLowersIntake) {
            lowerIntakeCommand().schedule();
          } else {
            raiseIntakeCommand().schedule();
          }
          m_xPressLowersIntake = !m_xPressLowersIntake;
        }));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .toggleOnTrue(m_intake.feedPercentCommand(0.5));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> {
          if (m_aPressStartsShooter) {
            shootFromDashboardDistanceCommand().schedule();
          } else {
            m_shooter.stopAll();
          }
          m_aPressStartsShooter = !m_aPressStartsShooter;
        }, m_shooter));

    new Trigger(() -> m_driverController.getPOV() == 180)
        .onTrue(new InstantCommand(
            () -> {
              m_fieldRelativeEnabled = !m_fieldRelativeEnabled;
              SmartDashboard.putBoolean("Drive Field Relative Enabled", m_fieldRelativeEnabled);
            }));

    new Trigger(() -> m_driverController.getPOV() == 0)
        .onTrue(new InstantCommand(m_cameraServerWrapper::toggleReadEnabled));
  }

  // when running a command for autonomous, call this to get the command
  public Command getAutonomousCommand() {
    //pathplanner
    return autoChooser.getSelected();
  }

  public void setAutoPidMode(boolean useLowPid){
    m_robotDrive.setAutoPidMode(useLowPid);
  }

  public void periodic() {
    m_cameraServerWrapper.periodic();
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
