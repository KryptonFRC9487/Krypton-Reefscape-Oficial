package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.IMotorController;
import frc.lib.SparkMController;


public class Subsystem extends SubsystemBase {

    public IMotorController motor;
    public SparkMax teste;

    public Subsystem(){

        motor = new SparkMController(0);
        motor.setPower(0);
        motor.setBrake(true);
    }
    
}
