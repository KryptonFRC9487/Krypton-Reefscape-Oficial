package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.ClimberMotor;


public class ClimberSubsystem extends SubsystemBase{

    private final SparkMax climberMotor;
    private final SparkMaxConfig climberConfig;
    private final RelativeEncoder climberEncoder;
    private final PIDController pid;
    

public ClimberSubsystem(){

    climberMotor = new SparkMax(ClimberMotor.kClimberMotor, MotorType.kBrushless);
    climberConfig = new SparkMaxConfig();
    
    climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    climberConfig.idleMode(IdleMode.kBrake);
    climberEncoder = climberMotor.getEncoder();

    climberEncoder.setPosition(0);

    pid = new PIDController(0, 0, 0);
}

@Override
public void periodic(){

    double output = pid.calculate(climberEncoder.getPosition());
    output = MathUtil.clamp(output, -0.3, 0.3); //0.4
    climberMotor.set(output);

    SmartDashboard.putNumber("Climber Position", climberEncoder.getPosition());
}

public void setClimberPosition(double setpoint){
    pid.setSetpoint(setpoint);
}

//teste
public void setClimberSpeed(double speed) {
    climberMotor.set(speed);
  }
}
