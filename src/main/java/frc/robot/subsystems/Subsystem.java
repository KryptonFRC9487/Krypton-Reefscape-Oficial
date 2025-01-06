package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorId;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Subsystem extends SubsystemBase{

    public static SparkMax test;
    
    public Subsystem(){
    
        test = new SparkMax(MotorId.motortest,MotorType.kBrushless);

    }

    public void TestSpeed(double speed){
        test.set(speed);
    }
}

