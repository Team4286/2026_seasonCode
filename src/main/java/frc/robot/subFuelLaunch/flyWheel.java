package frc.robot.subFuelLaunch;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Flywheel subsystem with a main shooter motor and a feed motor.
// Main flywheel uses closed-loop velocity control; feed uses open-loop percent output.
public class flyWheel extends SubsystemBase {
    // Closed-loop gains for flywheel velocity control (Spark internal PID).
    private static final double kFlywheelkP = 0.0002;
    private static final double kFlywheelkI = 0.0;
    private static final double kFlywheelkD = 0.0;

    // Default feed output used while actively shooting.
    private static final double kDefaultFeedPercent = 0.35;
    // Reverse output used during post-shot clear/jam clear.
    private static final double kDefaultClearPercent = -0.2;
    // Default duration to run reverse clear.
    private static final double kDefaultClearDurationSec = 1.0;
    // NEO free speed used for simple RPM->percent conversion helper.
    private static final double kNeoFreeSpeedRpm = 5676.0;

    // Main shooter motor (controls exit velocity).
    private final SparkMax m_flyWheelSpark;
    // Feed motor (pushes game pieces into shooter).
    private final SparkMax m_feedSpark;

    // Velocity feedback from shooter motor.
    private final RelativeEncoder m_flyWheelEncoder;
    // Velocity feedback from feed motor.
    private final RelativeEncoder m_feedEncoder;

    // Closed-loop interface for shooter velocity setpoint.
    private final SparkClosedLoopController m_flyWheelClosedLoopController;

    // Non-blocking timer for feed reverse clear behavior.
    private final Timer m_clearTimer = new Timer();

    // Requested shooter speed setpoint (RPM).
    private double m_targetFlywheelRpm = 0.0;
    // Requested forward feed output while shooting.
    private double m_feedPercent = 0.0;
    // Reverse output and duration for clear cycle.
    private double m_clearPercent = kDefaultClearPercent;
    private double m_clearDurationSec = kDefaultClearDurationSec;

    // State flags used by periodic to arbitrate feed behavior.
    private boolean m_isShooting = false;
    private boolean m_isClearingFeed = false;

    public flyWheel(int flyWheelCANId, int feedCANId) {
        //set motors
        m_flyWheelSpark = new SparkMax(flyWheelCANId, MotorType.kBrushless);
        m_feedSpark = new SparkMax(feedCANId, MotorType.kBrushless);

        //set the encoders
        m_flyWheelEncoder = m_flyWheelSpark.getEncoder();
        m_feedEncoder = m_feedSpark.getEncoder();

        // Configure the flywheel motor for closed-loop velocity control and feed motor for open-loop control.
        m_flyWheelClosedLoopController = m_flyWheelSpark.getClosedLoopController();

        // set up the configuration of the spark max for both motors, including idle mode and current limits. The flywheel gets a higher current limit since it needs more torque.
        // we also want the flywheel to coast when idle to allow it to spin down naturally, while the feed motor should brake to stop quickly when not feeding.
        SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        flywheelConfig
                .idleMode(IdleMode.kCoast)
                // Higher current limit for the primary flywheel motor.
                .smartCurrentLimit(60);
        flywheelConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(kFlywheelkP, kFlywheelkI, kFlywheelkD)
                .outputRange(-1.0, 1.0);
        m_flyWheelSpark.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig feedConfig = new SparkMaxConfig();
        feedConfig
                .idleMode(IdleMode.kBrake)
                // Lower current limit is usually enough for feed/index.
                .smartCurrentLimit(40);
        m_feedSpark.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Sets flywheel target speed in RPM with closed-loop velocity control.
    public void setFlyWheelSpeedRPM(double targetRPM) {
        m_targetFlywheelRpm = Math.max(0.0, targetRPM);
        m_flyWheelClosedLoopController.setSetpoint(m_targetFlywheelRpm, ControlType.kVelocity);
    }

    // Sets feed motor output in open-loop percent [-1.0, 1.0].
    public void setFeedPercent(double percentOutput) {
        m_feedPercent = MathUtil.clamp(percentOutput, -1.0, 1.0);
        if (m_isShooting && !m_isClearingFeed) {
            m_feedSpark.set(m_feedPercent);
        }
    }

    // Starts shooting with a flywheel target and optional feed percentage.
    public void startShooting(double flywheelTargetRpm, double feedPercent) {
        m_isShooting = true;
        m_isClearingFeed = false;
        m_clearTimer.stop();

        setFlyWheelSpeedRPM(flywheelTargetRpm);
        setFeedPercent(feedPercent);
        m_feedSpark.set(m_feedPercent);
    }

    // Starts shooting with default feed speed.
    public void startShooting(double flywheelTargetRpm) {
        startShooting(flywheelTargetRpm, kDefaultFeedPercent);
    }

    // Starts shooting using distance-based RPM and feed recommendations.
    public void startShootingForDistanceMeters(double distanceM) {
        double targetRpm = physicsCalc.requiredFlywheelRpmWithLut(distanceM);
        if (Double.isNaN(targetRpm)) {
            stopAll();
            return;
        }
        double feedPercent = dataTable.feedPercentForDistance(distanceM);
        startShooting(targetRpm, feedPercent);
    }

    // Distance convenience method where input is centimeters.
    public void startShootingForDistanceCm(double distanceCm) {
        startShootingForDistanceMeters(distanceCm / 100.0);
    }

    // Stops shooter and schedules feed reverse to clear jams.
    public void stopShooting() {
        m_isShooting = false;
        m_targetFlywheelRpm = 0.0;
        m_flyWheelSpark.stopMotor();

        m_isClearingFeed = true;
        m_clearTimer.restart();
    }

    // Backwards-compatible name used in earlier drafts.
    public void stopFlyWheel() {
        m_targetFlywheelRpm = 0.0;
        m_flyWheelSpark.stopMotor();
    }

    // Backwards-compatible name used in earlier drafts.
    public void stopFeedWheel() {
        m_feedSpark.stopMotor();
    }

    // Backwards-compatible API: converts feed RPM request to percent output.
    public void setFeedSpeedRPM(double targetRPM) {
        // Approximation only: uses free-speed normalization.
        setFeedPercent(targetRPM / kNeoFreeSpeedRpm);
    }

    // Backwards-compatible API: schedules a non-blocking reverse clear.
    public void reverseFeedWheel(double durationSeconds) {
        m_isClearingFeed = true;
        m_clearDurationSec = Math.max(0.0, durationSeconds);
        m_clearTimer.restart();
    }

    // Hard stop for both motors without clearing.
    public void stopAll() {
        m_isShooting = false;
        m_isClearingFeed = false;
        m_targetFlywheelRpm = 0.0;

        m_clearTimer.stop();
        m_flyWheelSpark.stopMotor();
        m_feedSpark.stopMotor();
    }

    public double getFlyWheelSpeedRPM() {
        return m_flyWheelEncoder.getVelocity();
    }

    public double getFeedSpeedRPM() {
        return m_feedEncoder.getVelocity();
    }

    public double getTargetFlywheelRPM() {
        return m_targetFlywheelRpm;
    }

    public boolean isShooting() {
        return m_isShooting;
    }

    public boolean isClearingFeed() {
        return m_isClearingFeed;
    }

    public void setClearBehavior(double clearPercent, double clearDurationSec) {
        m_clearPercent = MathUtil.clamp(clearPercent, -1.0, 1.0);
        m_clearDurationSec = Math.max(0.0, clearDurationSec);
    }

    @Override
    public void periodic() {
        // Clear state has priority over normal shooting feed.
        if (m_isClearingFeed) {
            if (m_clearTimer.hasElapsed(m_clearDurationSec)) {
                m_isClearingFeed = false;
                m_clearTimer.stop();
                m_feedSpark.stopMotor();
            } else {
                m_feedSpark.set(m_clearPercent);
            }
            return;
        }

        if (!m_isShooting) {
            // Keep feed stopped when idle.
            m_feedSpark.stopMotor();
        }
    }
}
