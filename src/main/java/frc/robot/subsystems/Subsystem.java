package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.SparkMController;


public class Subsystem extends SubsystemBase {

    public SparkMController motor, motor2;

    public Subsystem(){
        // motor = new SparkMController();
        // motor2 = new SparkMController();
        // motor.setBrake(true);
        // motor.setInverted(true);
        // motor.follow(motor2);
    }
}
