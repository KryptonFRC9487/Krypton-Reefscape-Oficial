package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.OuttakeConstants.OuttakePose;
import frc.robot.utils.SubsystemTracker;

public class OuttakeSubsystem extends SubsystemBase {

  private final SparkMax leftPivotMotor, rightPivotMotor, outtakeMotor;
  private final SparkMaxConfig leftPivotConfig, rightPivotConfig;
  private final DutyCycleEncoder encoder;
  private final DigitalInput m_limitSwitch;
  public static boolean limitSwitch;
  private final SubsystemTracker subsystemTracker;
  private OuttakePose outtakePose = OuttakePose.INIT;

  private final PIDController pid;

  public OuttakeSubsystem(SubsystemTracker subsystemTracker) {
    this.subsystemTracker = subsystemTracker;

    leftPivotMotor = new SparkMax(OuttakeConstants.LEFT_PIVOT_ID, MotorType.kBrushless);
    rightPivotMotor = new SparkMax(OuttakeConstants.RIGHT_PIVOT_ID, MotorType.kBrushless);

    leftPivotConfig = new SparkMaxConfig();
    rightPivotConfig = new SparkMaxConfig();

    leftPivotConfig.idleMode(IdleMode.kBrake);
    rightPivotConfig.idleMode(IdleMode.kBrake).inverted(true);

    leftPivotMotor.configure(leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightPivotMotor.configure(rightPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    outtakeMotor = new SparkMax(OuttakeConstants.OUTTAKE_ID, MotorType.kBrushless);

    m_limitSwitch = new DigitalInput(3);

    encoder = new DutyCycleEncoder(0, 360, 79.0);

    pid = new PIDController(
        OuttakeConstants.kP,
        OuttakeConstants.kI,
        OuttakeConstants.kD);

    pid.setSetpoint(OuttakePose.INIT.value);
    pid.setTolerance(0.5);
    pid.reset();
  }

  @Override
  public void periodic() {
    subsystemTracker.updateOuttakeRealPosition(outtakePose, getMeasurement());
    
    switch (subsystemTracker.getElevatorPose()) {
      case INITAL:
        if (subsystemTracker.getElevatorRealPosition() < 30.0)
          setOuttakePosition(OuttakePose.INIT);
        break;

      case L3:
        setOuttakePosition(OuttakePose.INIT);
      case L4:
        setOuttakePosition(OuttakePose.DEPOSIT);
        break;
    
      default:
        break;
    }

    double position = getMeasurement();

    double pidOutput = pid.calculate(position);
    double output = pidOutput;

    SmartDashboard.putNumber("O. Current Pos", position);
    SmartDashboard.putNumber("O. PID Output", pidOutput);
    SmartDashboard.putNumber("O. Output", output);
    SmartDashboard.putNumber("O. Setpoint", pid.getSetpoint());
    SmartDashboard.putNumber("O. Left Power", leftPivotMotor.get());
    SmartDashboard.putNumber("O. Right Power", rightPivotMotor.get());

    // Aplica a saída (descomente quando for necessário testar o movimento)
    // output = MathUtil.clamp(output, -0.17, 0.09);
    output = MathUtil.clamp(output, -0.17, 0.09);
    
    leftPivotMotor.set(output);
    rightPivotMotor.set(output);
  }

  // Função para obter a medição da posição (encoder)
  public double getMeasurement() {
    double rawAngle = encoder.get(); 

    return (rawAngle > 180) ? rawAngle - 360 : rawAngle;
  }

  // Função para definir a posição do outtake
  public void setOuttakePosition(OuttakePose outtakePose) {
    this.outtakePose = outtakePose;
    
    pid.setSetpoint(outtakePose.value);
  }

  // Função para definir a velocidade do outtake
    public void setOuttakeSpeed(double speed) {
      outtakeMotor.set(speed);
    }

  public boolean outtakeHasCoral() {
    return m_limitSwitch.get();
  }
}
