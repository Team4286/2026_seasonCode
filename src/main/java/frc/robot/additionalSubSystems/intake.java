package frc.robot.additionalSubSystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
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
// sets up an intake 
public class intake {
    
    //put variables here two motors, one for the roller and one for the arm
    //

   private final SparkMax intakeAxil;



    public intake(int intakeAxilCANId){
         intakeAxil = new SparkMax(intakeAxilCANId, MotorType.kBrushless);
        //initialize motors here
    }
    //conner is gay
     //put methods here, one to run the roller and one to move the arm
     public void runRoller(double speed){
        //set roller motor to speed
     }
     public void moveArm(double position){
        //set arm motor to position
     }
}
