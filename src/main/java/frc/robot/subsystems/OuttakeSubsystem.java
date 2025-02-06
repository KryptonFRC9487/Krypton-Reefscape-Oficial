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
import frc.robot.utils.SubsystemTracker;

public class OuttakeSubsystem extends SubsystemBase {

  private final SparkMax leftPivotMotor, rightPivotMotor, outtakeMotor;
  private final SparkMaxConfig leftPivotConfig, rightPivotConfig;
  private final DutyCycleEncoder encoder;
  private final DigitalInput m_limitSwitch;
  public static boolean limitSwitch;
  private final SubsystemTracker subsystemTracker;
  private OuttakePose outtakePose = OuttakePose.INIT;

  private final ProfiledPIDController pid;
  private final ArmFeedforward feedforward = new ArmFeedforward(
      OuttakeConstants.kS,
      OuttakeConstants.kG,
      OuttakeConstants.kV,
      OuttakeConstants.kA);

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

    encoder = new DutyCycleEncoder(0, 360, 72);  

    pid = new ProfiledPIDController(
        OuttakeConstants.kP,
        OuttakeConstants.kI,
        OuttakeConstants.kD,
        OuttakeConstants.TRAPEZOID_CONSTRAINTS);

    pid.setGoal(OuttakePose.INIT.value);
    pid.setTolerance(0.5);
  }

  @Override
  public void periodic() {
    subsystemTracker.updateOuttakeRealPosition(outtakePose, getMeasurement());
    
    switch (subsystemTracker.getElevatorPose()) {
      case INITAL:
        if (subsystemTracker.getElevatorRealPosition() < 30.0)
          setOuttakePosition(OuttakePose.INIT);
        break;

      case L2:
        setOuttakePosition(OuttakePose.MIDL2);
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
    double velocity = pid.getSetpoint().velocity;

    double pidOutput = pid.calculate(position);
    double feedforwardOutput = feedforward.calculate(position, velocity);
    double output = pidOutput + feedforwardOutput;

    output = MathUtil.clamp(output, -0.18, 0.09);

    SmartDashboard.putNumber("O. Current Pos Radians", position);
    SmartDashboard.putNumber("O. Current Pos Degrees", Math.toDegrees(position));
    SmartDashboard.putNumber("O. PID Output", pidOutput);
    SmartDashboard.putNumber("O. Output", output);
    SmartDashboard.putNumber("O. Setpoint Radians", pid.getSetpoint().position);
    SmartDashboard.putNumber("O. Setpoint Degrees", Math.toDegrees(pid.getSetpoint().position));
    SmartDashboard.putNumber("O. Left Power", leftPivotMotor.get());
    SmartDashboard.putNumber("O. Right Power", rightPivotMotor.get());

    leftPivotMotor.set(output);
    rightPivotMotor.set(output);
  }

  // Função para obter a medição da posição (encoder)
  public double getMeasurement() {
    double rawAngle = encoder.get(); 
    double angleInDegrees = (rawAngle > 180) ? rawAngle - 360 : rawAngle;

    return Math.toRadians(angleInDegrees);
  }

  // Função para definir a posição do outtake
  public void setOuttakePosition(OuttakePose outtakePose) {
    this.outtakePose = outtakePose;
    
    pid.setGoal(outtakePose.value);
  }

  // Função para definir a velocidade do outtake
    public void setOuttakeSpeed(double speed) {
      outtakeMotor.set(speed);
    }

  public boolean outtakeHasCoral() {
    return m_limitSwitch.get();
  }
}
