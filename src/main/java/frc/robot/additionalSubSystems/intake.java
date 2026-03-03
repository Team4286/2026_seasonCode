package frc.robot.additionalSubSystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

// Intake subsystem template:
// - axle motor: extends/retracts intake with 75:1 reduction
// - feed motor: constant feed roller with 4:1 reduction
public class intake extends SubsystemBase {
    private final SparkMax m_intakeAxle;
    private final SparkMax m_feedMotor;

    private final RelativeEncoder m_intakeAxleEncoder;
    private final SparkClosedLoopController m_intakeAxleClosedLoopController;

    private final SparkLimitSwitch m_forwardLimitSwitch;
    private final SparkLimitSwitch m_reverseLimitSwitch;
    private final DigitalInput m_forwardLimitDio;
    private final DigitalInput m_reverseLimitDio;

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

        LimitSwitchConfig.Type switchType = IntakeConstants.kIntakeLimitSwitchNormallyClosed
                ? LimitSwitchConfig.Type.kNormallyClosed
                : LimitSwitchConfig.Type.kNormallyOpen;
        if (IntakeConstants.kUseSparkMaxLimitSwitches) {
            axleConfig.limitSwitch
                    .forwardLimitSwitchType(switchType)
                    .reverseLimitSwitchType(switchType)
                    .forwardLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor)
                    .reverseLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor);
        } else {
            axleConfig.limitSwitch
                    .forwardLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kKeepMovingMotor)
                    .reverseLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kKeepMovingMotor);
        }

        m_intakeAxle.configure(axleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig feedConfig = new SparkMaxConfig();
        feedConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(IntakeConstants.kIntakeFeedCurrentLimitAmps)
                .inverted(IntakeConstants.kIntakeFeedInverted);
        m_feedMotor.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (IntakeConstants.kUseSparkMaxLimitSwitches) {
            m_forwardLimitSwitch = m_intakeAxle.getForwardLimitSwitch();
            m_reverseLimitSwitch = m_intakeAxle.getReverseLimitSwitch();
            m_forwardLimitDio = null;
            m_reverseLimitDio = null;
        } else {
            m_forwardLimitSwitch = null;
            m_reverseLimitSwitch = null;
            m_forwardLimitDio = new DigitalInput(IntakeConstants.kIntakeForwardLimitDioChannel);
            m_reverseLimitDio = new DigitalInput(IntakeConstants.kIntakeReverseLimitDioChannel);
        }
    }

    public void setFeedPercent(double percentOutput) {
        m_feedMotor.set(MathUtil.clamp(percentOutput, -1.0, 1.0));
    }

    public void setAxlePercent(double percentOutput) {
        double clamped = MathUtil.clamp(percentOutput, -1.0, 1.0);
        if ((clamped > 0.0 && isForwardLimitPressed()) || (clamped < 0.0 && isReverseLimitPressed())) {
            m_intakeAxle.stopMotor();
            return;
        }
        m_intakeAxle.set(clamped);
    }

    public void setAxlePositionRotations(double outputRotations) {
        double target = MathUtil.clamp(
                outputRotations,
                IntakeConstants.kIntakeAxleMinRotations,
                IntakeConstants.kIntakeAxleMaxRotations);
        m_intakeAxleClosedLoopController.setSetpoint(target, ControlType.kPosition);
    }

    public void stopAll() {
        m_intakeAxle.stopMotor();
        m_feedMotor.stopMotor();
    }

    public void stopAxle() {
        m_intakeAxle.stopMotor();
    }

    public void stopFeed() {
        m_feedMotor.stopMotor();
    }

    public boolean isForwardLimitPressed() {
        return IntakeConstants.kUseSparkMaxLimitSwitches
                ? m_forwardLimitSwitch.isPressed()
                : isDioLimitPressed(m_forwardLimitDio);
    }

    public boolean isReverseLimitPressed() {
        return IntakeConstants.kUseSparkMaxLimitSwitches
                ? m_reverseLimitSwitch.isPressed()
                : isDioLimitPressed(m_reverseLimitDio);
    }

    private boolean isDioLimitPressed(DigitalInput input) {
        // Assumes the switch is wired between DIO signal and GND with roboRIO pull-up.
        // NO: pressed -> circuit closes -> false.
        // NC: pressed -> circuit opens -> true.
        return input.get() == IntakeConstants.kIntakeLimitSwitchNormallyClosed;
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
        SmartDashboard.putBoolean("Intake/ForwardLimitPressed", isForwardLimitPressed());
        SmartDashboard.putBoolean("Intake/ReverseLimitPressed", isReverseLimitPressed());
        SmartDashboard.putBoolean("Intake/UsingSparkMaxLimits", IntakeConstants.kUseSparkMaxLimitSwitches);
        SmartDashboard.putNumber("Intake/AxlePositionRotations", getAxlePositionRotations());
    }
}
