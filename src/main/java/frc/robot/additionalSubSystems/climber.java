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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

// AndyMark 2-hook stage-1 climber on a REV NEO with 100:1 reduction.
// No limit switches are configured yet, so motion is software-clamped.
public class climber extends SubsystemBase {

    // motor canID
    private final SparkMax m_climberMotor;
   
    // encoder type 
    private final RelativeEncoder m_climberEncoder;
    private final SparkClosedLoopController m_climberClosedLoopController;
   
   
    public climber() {
        m_climberMotor = new SparkMax(ClimberConstants.kClimberCanId, MotorType.kBrushless);
        m_climberEncoder = m_climberMotor.getEncoder();
        m_climberClosedLoopController = m_climberMotor.getClosedLoopController();

        SparkMaxConfig climberConfig = new SparkMaxConfig();
        climberConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ClimberConstants.kClimberCurrentLimitAmps)
                .inverted(ClimberConstants.kClimberInverted);
        climberConfig.encoder
                // Convert motor rotations to output axle rotations (100:1 reduction).
                .positionConversionFactor(1.0 / ClimberConstants.kClimberMotorToOutputRatio)
                .velocityConversionFactor(1.0 / (60.0 * ClimberConstants.kClimberMotorToOutputRatio));
        climberConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ClimberConstants.kClimberkP, ClimberConstants.kClimberkI, ClimberConstants.kClimberkD)
                .outputRange(-1.0, 1.0);
        m_climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setClimberPercent(double percentOutput) {
        m_climberMotor.set(MathUtil.clamp(percentOutput, -1.0, 1.0));
    }

    public void setClimberPositionRotations(double outputRotations) {
        double target = MathUtil.clamp(
                outputRotations,
                ClimberConstants.kClimberMinRotations,
                ClimberConstants.kClimberMaxRotations);
        m_climberClosedLoopController.setSetpoint(target, ControlType.kPosition);
    }

    public void stopClimber() {
        m_climberMotor.stopMotor();
    }

    public double getClimberPositionRotations() {
        return m_climberEncoder.getPosition();
    }

    public void zeroClimberAtRetractedPosition() {
        m_climberEncoder.setPosition(ClimberConstants.kClimberMinRotations);
    }

}
