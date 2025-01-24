package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeSubsystem extends SubsystemBase {

  private final SparkMax leftPivotMotor, rightPivotMotor, outtakeMotor;
  private final SparkMaxConfig leftPivotConfig, rightPivotConfig;
  private final DutyCycleEncoder encoder;
  private final DigitalInput m_limitSwitch;

  private final ProfiledPIDController pid;
  private final ArmFeedforward feedforward;

  public static boolean limitSwitch;

  public OuttakeSubsystem() {

    leftPivotMotor = new SparkMax(OuttakeConstants.LEFT_PIVOT_ID, MotorType.kBrushless);
    rightPivotMotor = new SparkMax(OuttakeConstants.RIGHT_PIVOT_ID, MotorType.kBrushless);

    leftPivotConfig = new SparkMaxConfig();
    rightPivotConfig = new SparkMaxConfig();

    leftPivotConfig.idleMode(IdleMode.kBrake);
    rightPivotConfig.idleMode(IdleMode.kBrake);

    rightPivotConfig.follow(leftPivotMotor, true);

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

    output = MathUtil.clamp(output, -1.0, 1.0);

    leftPivotMotor.set(output);

    SmartDashboard.putNumber("Encoder Outtake", position);
    SmartDashboard.putNumber("PID Output", pidOutput);
    SmartDashboard.putNumber("Feedforward Output", feedforwardOutput);
    SmartDashboard.putNumber("Total Output", output);
  }

  public double getMeasurement() {
    return encoder.get();
  }

  public void setOuttakePosition(double setpoint) {
    pid.setGoal(setpoint);
  }

  public void setOuttakeSpeed(double speed) {
    outtakeMotor.set(speed);
  }
}
