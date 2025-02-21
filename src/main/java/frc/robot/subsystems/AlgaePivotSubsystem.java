package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaePivotSubsystem extends SubsystemBase{

    private TalonFX kraken;
    private double encoder;

    private final PIDController pidc;


public AlgaePivotSubsystem(){

    kraken = new TalonFX(0);
    kraken.setNeutralMode(NeutralModeValue.Brake);
    kraken.setPosition(0);

    pidc = new PIDController(0.1, 0, 0);
    
}


@Override
public void periodic(){

    double output = pidc.calculate(encoder);
    output = MathUtil.clamp(output, -0.3, 0.3); //0.4
    kraken.set(output);

    encoder = kraken.getPosition().getValueAsDouble();

    SmartDashboard.putNumber("Kraken", encoder);

}

public void setKrakenSpeed(double speed) {
    kraken.set(speed);
  }

public void setKrakenPosition(double setpoint){
    pidc.setSetpoint(setpoint);
}
}
