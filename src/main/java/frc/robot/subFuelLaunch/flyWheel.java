package frc.robot.subFuelLaunch;

// REV Robotics imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;


// this class represents the flywheel subsystem of the robot
public class flyWheel {

    // Flywheel motor controller
    private final SparkMax m_flyWheelSpark;
    // it is assumed that we will use a brushless motor for the flywheel.
    // a fly wheel encoder needs to measure the rotational speed of the flywheel
    // but not its absolute position
    private final RelativeEncoder m_flyWheelEncoder;
    //adjust the speed of the flywheel using closed loop control
    private final SparkClosedLoopController m_flyWheelClosedLoopController;

    // Constructor for the flywheel subsystem
    public flyWheel (int flyWheelCANId) {
        m_flyWheelSpark = new SparkMax(flyWheelCANId, com.revrobotics.spark.SparkBase.MotorType.kBrushless);
        m_flyWheelEncoder = m_flyWheelSpark.getEncoder();
        m_flyWheelClosedLoopController = m_flyWheelSpark.getClosedLoopController();
    }

    // Method to set the flywheel speed in RPM using closed loop control

    // targetRPM: desired speed in revolutions per minute

    //this will recieve the requiredVelocityMpsFromCm method from physicsCalc class in a later file
    public void setFlyWheelSpeedRPM(double targetRPM) {
        m_flyWheelClosedLoopController.setSetpoint(targetRPM, ControlType.kVelocity);
    }

    // Method to get the current flywheel speed in RPM
    public double getFlyWheelSpeedRPM() {
        return m_flyWheelEncoder.getVelocity(); 
    }

    // Method to stop the flywheel
    public void stopFlyWheel() {
        m_flyWheelSpark.stopMotor();
    }
}
