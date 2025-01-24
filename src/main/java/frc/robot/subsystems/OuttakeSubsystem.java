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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OuttakeConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class OuttakeSubsystem extends SubsystemBase {

  private final SparkMax leftPivotMotor, rightPivotMotor, outtakeMotor;
  private final SparkMaxConfig leftPivotConfig, rightPivotConfig;
  private final ProfiledPIDController pid;
  private final DutyCycleEncoder encoder;
  private final DigitalInput m_limitSwitch;

  private final ArmFeedforward feedforward = new ArmFeedforward(
      OuttakeConstants.kS,
      OuttakeConstants.kG,
      OuttakeConstants.kV,
      OuttakeConstants.kA);

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutAngle m_distance = Rotations.mutable(0);
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

  // Adicionando rotina SysId
  // private final SysIdRoutine routine = new SysIdRoutine(
  // new SysIdRoutine.Config(),
  // new SysIdRoutine.Mechanism(
  // voltage -> leftPivotMotor.setVoltage(voltage),
  // log -> {
  // log.motor("leftPivotMotor")
  // .voltage(m_appliedVoltage.mut_replace(
  // leftPivotMotor.get() * 12.0, // Considera a bateria como 12V nominal
  // Volts))
  // .angularPosition(m_distance.mut_replace(
  // leftPivotMotor.getEncoder().getPosition(), Rotations))
  // .angularVelocity(m_velocity.mut_replace(
  // leftPivotMotor.getEncoder().getVelocity(), RadiansPerSecond));
  // },
  // this));

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

    pid = new ProfiledPIDController(
        OuttakeConstants.kP,
        OuttakeConstants.kI,
        OuttakeConstants.kD,
        OuttakeConstants.MOVEMENT_CONSTRAINTS);

    encoder = new DutyCycleEncoder(0);
  }

  @Override
  public void periodic() {
    double output = pid.calculate(getMeasurement());
    output = MathUtil.clamp(output, -0.5, 0.1);
    leftPivotMotor.set(output);

    SmartDashboard.putNumber("Encoder Outtake", getMeasurement());
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

  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  // return routine.quasistatic(direction);
  // }

  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  // return routine.dynamic(direction);
  // }
}
