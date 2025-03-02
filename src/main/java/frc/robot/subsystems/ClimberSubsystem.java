package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberMotor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class ClimberSubsystem extends SubsystemBase{

    SparkMax climberMotor;
    SparkMaxConfig climberConfig;


public ClimberSubsystem(){


    climberMotor = new SparkMax(ClimberMotor.m_climber,MotorType.kBrushless);
    climberConfig = new SparkMaxConfig();

    climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    climberConfig.idleMode(IdleMode.kBrake);
}

public void setClimberSpeed(double speed){
    climberMotor.set(speed);
}
    
}
