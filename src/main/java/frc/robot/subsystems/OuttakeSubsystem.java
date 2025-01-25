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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.OuttakeConstants.OuttakePose;

public class OuttakeSubsystem extends SubsystemBase {

  private final SparkMax leftPivotMotor, rightPivotMotor, outtakeMotor;
  private final SparkMaxConfig leftPivotConfig, rightPivotConfig;
  private final DutyCycleEncoder encoder;
  private final DigitalInput m_limitSwitch;
  public static boolean limitSwitch;

  private final ProfiledPIDController pid;
  private final ArmFeedforward feedforward;

  public OuttakeSubsystem() {
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

    encoder = new DutyCycleEncoder(0);

    pid = new ProfiledPIDController(
        OuttakeConstants.kP,
        OuttakeConstants.kI,
        OuttakeConstants.kD,
        OuttakeConstants.TRAPEZOID_CONSTRAINTS);

    pid.setGoal(OuttakePose.DEPOSIT.value);
    pid.setTolerance(0.5);

    feedforward = new ArmFeedforward(
        OuttakeConstants.kS,
        OuttakeConstants.kG,
        OuttakeConstants.kV,
        OuttakeConstants.kA);
  }

  @Override
  public void periodic() {
    double position = getMeasurement();
    double velocity = pid.getSetpoint().velocity;

    double pidOutput = pid.calculate(position);
    double feedforwardOutput = feedforward.calculate(position, velocity);
    double output = pidOutput + feedforwardOutput;

    SmartDashboard.putNumber("O. Current Pos", position);
    SmartDashboard.putNumber("O. PID Output", pidOutput);
    SmartDashboard.putNumber("O. FF Output", feedforwardOutput);
    SmartDashboard.putNumber("O. Output", output);
    SmartDashboard.putNumber("O. Setpoint", pid.getSetpoint().position);
    SmartDashboard.putNumber("O. Left Power", leftPivotMotor.get());
    SmartDashboard.putNumber("O. Right Power", rightPivotMotor.get());

    // Aplica a saída (descomente quando for necessário testar o movimento)
    output = MathUtil.clamp(output, -0.05, 0.05);

    leftPivotMotor.set(output);
    rightPivotMotor.set(output);
  }

  // Função para obter a medição da posição (encoder)
  public double getMeasurement() {
    return encoder.get() * 360 - OuttakeConstants.OUTTAKE_ENCODER_OFFSET;
  }

  // Função para definir a posição do outtake
  public void setOuttakePosition(double setpoint) {
    pid.setGoal(setpoint);
  }

  // Função para definir a velocidade do outtake
  public void setOuttakeSpeed(double speed) {
    outtakeMotor.set(speed);
  }
}
