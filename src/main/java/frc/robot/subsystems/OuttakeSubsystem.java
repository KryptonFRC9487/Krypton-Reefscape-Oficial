package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OuttakeConstants.ArmConfig;
import frc.robot.Constants.OuttakeConstants.Gains;
import frc.robot.Constants.OuttakeConstants.HardwareConfig;
import frc.robot.Constants.OuttakeConstants.OuttakePose;
import frc.robot.Constants.OuttakeConstants.TrapezoidProfileConstants;
import frc.robot.utils.SubsystemTracker;

public class OuttakeSubsystem extends SubsystemBase {

  private final DCMotor armGearbox = DCMotor.getNEO(2);

  private final SparkMax m_leftPivotMotor, m_rightPivotMotor, m_outtakeMotor;
  private final SparkMaxConfig m_leftPivotConfig, m_rightPivotConfig;
  private final SparkClosedLoopController m_armPivotController;
  private final RelativeEncoder m_leftPivotEncoder, m_rightPivotEncoder;
  private final DutyCycleEncoder m_encoder;
  private final AbsoluteEncoder m_absoluteEncoder;

  private final DigitalInput m_limitSwitch;
  public static boolean limitSwitch;
  private final SubsystemTracker m_subsystemTracker;
  private OuttakePose outtakePose = OuttakePose.INIT;

  private final ProfiledPIDController m_pid;
  private final ArmFeedforward m_feedforward;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutAngle m_angle = Rotations.mutable(0);
  private final MutAngularVelocity m_velocity = RPM.mutable(0);

  private final SysIdRoutine m_sysIdRoutine;

  public final Trigger atMin = new Trigger(() -> getAngle().lte(ArmConfig.kMinAngle.plus(Degrees.of(15))));
  public final Trigger atMax = new Trigger(() -> getAngle().gte(ArmConfig.kMaxAngle.minus(Degrees.of(15))));

  public OuttakeSubsystem(SubsystemTracker subsystemTracker) {
    this.m_subsystemTracker = subsystemTracker;

    m_leftPivotMotor = new SparkMax(HardwareConfig.kLeftPivotId, MotorType.kBrushless);
    m_rightPivotMotor = new SparkMax(HardwareConfig.kRightPivotId, MotorType.kBrushless);
    m_outtakeMotor = new SparkMax(HardwareConfig.kOuttakeId, MotorType.kBrushless);

    m_leftPivotEncoder = m_leftPivotMotor.getEncoder();
    m_rightPivotEncoder = m_rightPivotMotor.getEncoder();
    m_absoluteEncoder = m_rightPivotMotor.getAbsoluteEncoder();
    m_encoder = new DutyCycleEncoder(HardwareConfig.kAbsoluteEncoderId, 360.0, HardwareConfig.kOuttakeEncoderOffset);
    // m_encoder.setInverted(true);

    m_feedforward = new ArmFeedforward(Gains.kS, Gains.kG, Gains.kV, Gains.kA);
    m_pid = new ProfiledPIDController(Gains.kP, Gains.kI, Gains.kD, TrapezoidProfileConstants.kConstraints);
    m_pid.setGoal(OuttakePose.INIT.value);
    // m_pid.setTolerance(0.1);

    m_limitSwitch = new DigitalInput(3);

    m_armPivotController = m_rightPivotMotor.getClosedLoopController();

    m_leftPivotConfig = new SparkMaxConfig();
    m_rightPivotConfig = new SparkMaxConfig();

    m_rightPivotConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ArmConfig.kStallCurrentLimit)
        .closedLoopRampRate(ArmConfig.kClosedLoopRate).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Gains.kP, Gains.kI, Gains.kD)
        .outputRange(-1.0, 1.0);
    // .outputRange(-0.05, 0.05);

    m_rightPivotConfig.encoder
        .positionConversionFactor(HardwareConfig.kGearRatio)
        .velocityConversionFactor(HardwareConfig.kGearRatio);

    m_rightPivotConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ArmConfig.kStallCurrentLimit)
        .closedLoopRampRate(ArmConfig.kClosedLoopRate).encoder
        .positionConversionFactor(HardwareConfig.kGearRatio)
        .velocityConversionFactor(HardwareConfig.kGearRatio);

    m_leftPivotConfig.follow(m_rightPivotMotor, true);

    m_leftPivotMotor.configure(m_leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightPivotMotor.configure(m_rightPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Timer.delay(2.0);

    synchronizeMotors();

    m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.per(Second).of(ArmConfig.kClosedLoopRate),
            Volts.of(1),
            Seconds.of(30)),
        new SysIdRoutine.Mechanism(
            m_rightPivotMotor::setVoltage,
            log -> {
              log.motor("arm-2")
                  .voltage(m_appliedVoltage
                      .mut_replace(m_rightPivotMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                  .angularPosition(
                      m_angle.mut_replace(m_rightPivotEncoder.getPosition(), Rotations))
                  .angularVelocity(
                      m_velocity.mut_replace(m_rightPivotEncoder.getVelocity(), RotationsPerSecond));
            },
            this));

    SmartDashboard.putData("Arm PID", m_pid);
  }

  private void synchronizeMotors() {
    Angle armAngle = Degrees.of(m_encoder.get()).times(-1.0);
    m_leftPivotEncoder.setPosition(armAngle.in(Rotations));
    m_rightPivotEncoder.setPosition(armAngle.in(Rotations));
  }

  @Override
  public void periodic() {
    m_subsystemTracker.updateOuttakeRealPosition(outtakePose, getMeasurement());

    switch (m_subsystemTracker.getElevatorPose()) {
      case INITAL:
        if (m_subsystemTracker.getElevatorRealPosition() < 30.0)
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

    double position = Degrees.of(getMeasurement()).in(Radians);
    double velocity = m_pid.getSetpoint().velocity;

    double pidOutput = m_pid.calculate(position);
    double output = pidOutput;

    double feedforwardOutput = m_feedforward.calculate(position, velocity);

    if (feedforwardOutput > 0)
      output += feedforwardOutput;

    m_rightPivotMotor.set(output);

  SmartDashboard.putNumber("Arm PID Output", pidOutput);
  SmartDashboard.putNumber("Arm FF Output", feedforwardOutput);
  SmartDashboard.putNumber("Arm Output", output);
  SmartDashboard.putNumber("Arm Position", Radians.of(position).in(Degrees));
  SmartDashboard.putNumber("Arm Velocity", velocity);
  SmartDashboard.putNumber("Arm Setpoint", Radians.of(outtakePose.value).in(Degrees));
  }

  // Função para obter a medição da posição (encoder)
  public double getMeasurement() {
    // return Rotations.of(m_rightPivotEncoder.getPosition()).in(Degree);
    double rawAngle = m_encoder.get();

    return -((rawAngle > 180) ? rawAngle - 360 : rawAngle);
  }

  // Função para definir a posição do outtake
  public void setOuttakePosition(OuttakePose outtakePose) {
    this.outtakePose = outtakePose;

    m_pid.setGoal(outtakePose.value);
  }

  // Função para definir a velocidade do outtake
  public void setOuttakeSpeed(double speed) {
    m_outtakeMotor.set(speed);
  }

  public boolean outtakeHasCoral() {
    return m_limitSwitch.get();
  }

  public Command runSysIdRoutine() {
    return m_sysIdRoutine.dynamic(Direction.kForward).until(atMax)
        .andThen(m_sysIdRoutine.dynamic(Direction.kReverse).until(atMin))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kForward).until(atMax))
        .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse).until(atMin));
  }

  public Angle getAngle() {
    return Degrees.of(getMeasurement());

    // m_angle.mut_replace(
    // RobotMath.Arm.convertSensorUnitsToAngle(m_angle.mut_replace(m_leftPivotEncoder.getPosition(),
    // Rotations)));

    // return m_angle;
  }
}
