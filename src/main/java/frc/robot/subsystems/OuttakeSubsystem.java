package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OuttakeConstants.HardwareConfig;

public class OuttakeSubsystem extends SubsystemBase {

  private final SparkMax m_outtakeMotor;
  private final DigitalInput m_limitSwitch;

  public OuttakeSubsystem() {
    m_outtakeMotor = new SparkMax(HardwareConfig.kOuttakeId, MotorType.kBrushless);
    m_limitSwitch = new DigitalInput(2);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Outtake Has Coral", outtakeHasCoral());
  }

  public void setOuttakeSpeed(double speed) {
    m_outtakeMotor.set(speed);
  }

  public Command setOuttakeSpeedCmd(double speed) {
    return run(() -> setOuttakeSpeed(speed));
  }

  public boolean outtakeHasCoral() {
    return !m_limitSwitch.get();
  }
}
