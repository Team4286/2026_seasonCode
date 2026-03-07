package frc.robot.additionalSubSystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

// Intake subsystem template:
// - axle motor: extends/retracts intake with 75:1 reduction
// - feed motor: constant feed roller with 4:1 reduction
public class intake extends SubsystemBase {
    // Hardware components.
    private final SparkMax m_intakeAxle;
    private final SparkMax m_feedMotor;
// - axle encoder and closed-loop controller for position control.
    private final RelativeEncoder m_intakeAxleEncoder;
    private final SparkClosedLoopController m_intakeAxleClosedLoopController;
    private double m_lastAxlePercentCommand = 0.0;

    // DIO limit switches on roboRIO.
    private final DigitalInput m_forwardLimitSwitch;
    private final DigitalInput m_reverseLimitSwitch;

    public intake() {
        m_intakeAxle = new SparkMax(IntakeConstants.kIntakeAxleCanId, MotorType.kBrushless);
        m_feedMotor = new SparkMax(IntakeConstants.kIntakeFeedCanId, MotorType.kBrushless);

        m_intakeAxleEncoder = m_intakeAxle.getEncoder();
        m_intakeAxleClosedLoopController = m_intakeAxle.getClosedLoopController();

        SparkMaxConfig axleConfig = new SparkMaxConfig();
        axleConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(IntakeConstants.kIntakeAxleCurrentLimitAmps)
                .inverted(IntakeConstants.kIntakeAxleInverted);
        axleConfig.encoder
                // Convert motor rotations to output axle rotations (75:1 reduction).
                .positionConversionFactor(1.0 / IntakeConstants.kIntakeAxleMotorToOutputRatio)
                .velocityConversionFactor(1.0 / (60.0 * IntakeConstants.kIntakeAxleMotorToOutputRatio));
        axleConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(IntakeConstants.kIntakeAxlekP, IntakeConstants.kIntakeAxlekI, IntakeConstants.kIntakeAxlekD)
                .outputRange(-1.0, 1.0);

        m_intakeAxle.configure(axleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig feedConfig = new SparkMaxConfig();
        feedConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(IntakeConstants.kIntakeFeedCurrentLimitAmps)
                .inverted(IntakeConstants.kIntakeFeedInverted);
        m_feedMotor.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_forwardLimitSwitch = new DigitalInput(IntakeConstants.kIntakeForwardLimitDioChannel);
        m_reverseLimitSwitch = new DigitalInput(IntakeConstants.kIntakeReverseLimitDioChannel);
    }

    public void setFeedPercent(double percentOutput) {
        if (isAtUpPosition() || isMovingTowardUp()) {
            m_feedMotor.stopMotor();
            return;
        }
        m_feedMotor.set(MathUtil.clamp(percentOutput, -1.0, 1.0));
    }

    public void setAxlePercent(double percentOutput) {
        double clamped = MathUtil.clamp(percentOutput, -1.0, 1.0);
        m_lastAxlePercentCommand = clamped;
        if (isMovingTowardUp()) {
            m_feedMotor.stopMotor();
        }
        if ((clamped > 0.0 && isForwardLimitPressed()) || (clamped < 0.0 && isReverseLimitPressed())) {
            m_intakeAxle.stopMotor();
            m_lastAxlePercentCommand = 0.0;
            return;
        }
        m_intakeAxle.set(clamped);
    }

    public void setAxlePositionRotations(double outputRotations) {
        double target = MathUtil.clamp(
                outputRotations,
                IntakeConstants.kIntakeAxleMinRotations,
                IntakeConstants.kIntakeAxleMaxRotations);
        m_lastAxlePercentCommand = 0.0;
        double positionError = target - getAxlePositionRotations();
        if ((positionError * IntakeConstants.kIntakeUpPercentDirection) > 0.0) {
            m_feedMotor.stopMotor();
        }
        m_intakeAxleClosedLoopController.setSetpoint(target, ControlType.kPosition);
    }

    public void stopAll() {
        m_intakeAxle.stopMotor();
        m_feedMotor.stopMotor();
    }

    public void stopAxle() {
        m_intakeAxle.stopMotor();
        m_lastAxlePercentCommand = 0.0;
    }

    public void stopFeed() {
        m_feedMotor.stopMotor();
    }

    public boolean isForwardLimitPressed() {
        return isDioLimitPressed(m_forwardLimitSwitch);
    }

    public boolean isReverseLimitPressed() {
        return isDioLimitPressed(m_reverseLimitSwitch);
    }

    private boolean isDioLimitPressed(DigitalInput limitSwitch) {
        // DIO reads true when open, false when shorted to GND.
        // For normally-closed switches: pressed -> open -> true.
        // For normally-open switches: pressed -> closed -> false.
        boolean dioState = limitSwitch.get();
        return IntakeConstants.kIntakeLimitSwitchNormallyClosed ? dioState : !dioState;
    }

    private boolean isAtUpPosition() {
        return IntakeConstants.kIntakeUpIsForwardLimit ? isForwardLimitPressed() : isReverseLimitPressed();
    }

    private boolean isMovingTowardUp() {
        return (m_lastAxlePercentCommand * IntakeConstants.kIntakeUpPercentDirection) > 0.0;
    }

    public double getAxlePositionRotations() {
        return m_intakeAxleEncoder.getPosition();
    }

    public void zeroAxlePositionAtRetractedLimit() {
        if (isReverseLimitPressed()) {
            m_intakeAxleEncoder.setPosition(IntakeConstants.kIntakeAxleMinRotations);
        }
    }

    // Named command helpers for later binding in RobotContainer.
    public Command feedPercentCommand(double percentOutput) {
        return Commands.runEnd(() -> setFeedPercent(percentOutput), this::stopFeed, this);
    }

    public Command axlePercentCommand(double percentOutput) {
        return Commands.runEnd(() -> setAxlePercent(percentOutput), this::stopAxle, this);
    }

    public Command holdAxlePositionCommand(double outputRotations) {
        return Commands.run(() -> setAxlePositionRotations(outputRotations), this);
    }

    @Override
    public void periodic() {
        // Intentionally left empty.
    }
}
