package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaePivotSubsystem extends SubsystemBase {

  private static final double GEAR_RATIO = 14.0 / 45.0; // Ajuste conforme sua relação de transmissão
  private static final double FEEDBACK_COEFFICIENT = 360.0 / (2048.0 * GEAR_RATIO);

  private TalonFX kraken;
  private final PIDController pidc;

  public AlgaePivotSubsystem() {

    kraken = new TalonFX(0);
    kraken.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    // configs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    configs.Feedback.SensorToMechanismRatio = FEEDBACK_COEFFICIENT;
    kraken.getConfigurator().apply(configs);

    kraken.setPosition(0);
    pidc = new PIDController(0.1, 0, 0);
  }

  @Override
  public void periodic() {

    double encoderDegrees = kraken.getPosition().getValueAsDouble(); // Já retorna em graus
    double output = pidc.calculate(encoderDegrees);
    output = MathUtil.clamp(output, -0.3, 0.3);
    // kraken.set(output);

    SmartDashboard.putNumber("Kraken Degrees", encoderDegrees);
  }

  public void setKrakenSpeed(double speed) {
    kraken.set(speed);
  }

  public void setKrakenPosition(double setpointDegrees) {
    pidc.setSetpoint(setpointDegrees);
  }
}

