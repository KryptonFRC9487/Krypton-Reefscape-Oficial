package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.OuttakeConstants.OuttakePose;
import frc.robot.commands.OuttakeCommand;
import frc.robot.utils.SubsystemTracker;

public class OuttakeSubsystem extends SubsystemBase {

  private final DCMotor armGearbox = DCMotor.getNEO(2);

  private final SparkMax leftPivotMotor, rightPivotMotor, outtakeMotor;
  private final SparkMaxConfig leftPivotConfig, rightPivotConfig;
  private final SparkClosedLoopController armPivotController;
  private final RelativeEncoder leftPivotEncoder, rightPivotEncoder;
  private final DutyCycleEncoder encoder;
  private final AbsoluteEncoder absoluteEncoder;

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

  // public final Trigger atMin = new Trigger(() ->
  // getAngle().lte(OuttakeConstants.ARM_MIN_ANGLE.plus(Degrees.of(5))));
  // public final Trigger atMax = new Trigger(() ->
  // getAngle().gte(OuttakeConstants.ARM_MAX_ANGLE.minus(Degrees.of(5))));

  public OuttakeSubsystem(SubsystemTracker subsystemTracker) {
    this.subsystemTracker = subsystemTracker;

    leftPivotMotor = new SparkMax(OuttakeConstants.kLeftPivotId, MotorType.kBrushless);
    rightPivotMotor = new SparkMax(OuttakeConstants.kRightPivotId, MotorType.kBrushless);

    leftPivotEncoder = leftPivotMotor.getEncoder();
    rightPivotEncoder = rightPivotMotor.getEncoder();

    armPivotController = leftPivotMotor.getClosedLoopController();

    absoluteEncoder = leftPivotMotor.getAbsoluteEncoder();

    leftPivotConfig = new SparkMaxConfig();
    rightPivotConfig = new SparkMaxConfig();

    leftPivotConfig.idleMode(IdleMode.kBrake).inverted(true)
        .smartCurrentLimit(OuttakeConstants.kArmStallCurrentLimit)
        .closedLoopRampRate(OuttakeConstants.kArmClosedLoopRate).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(OuttakeConstants.kP, OuttakeConstants.kI, OuttakeConstants.kD)
        .outputRange(-1.0, 1.0);

    leftPivotConfig.encoder
        .positionConversionFactor(1.0 / 15.0)
        .velocityConversionFactor(1.0 / 15.0);

    rightPivotConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(OuttakeConstants.kArmStallCurrentLimit)
        .closedLoopRampRate(OuttakeConstants.kArmClosedLoopRate).encoder
        .positionConversionFactor(1.0 / 15.0)
        .velocityConversionFactor(1.0 / 15.0);

    rightPivotConfig.follow(leftPivotMotor, true);

    leftPivotMotor.configure(leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightPivotMotor.configure(rightPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    outtakeMotor = new SparkMax(OuttakeConstants.kOuttakeId, MotorType.kBrushless);

    m_limitSwitch = new DigitalInput(3);

    encoder = new DutyCycleEncoder(0, 360.0, 72.0);
    encoder.setInverted(true);

    pid = new ProfiledPIDController(
        OuttakeConstants.kP,
        OuttakeConstants.kI,
        OuttakeConstants.kD,
        OuttakeConstants.kTrapezoidConstraints);

    pid.setGoal(OuttakePose.INIT.value);
    pid.setTolerance(0.5);

    Timer.delay(2.0);

    synchronizeMotors();
  }

  private void synchronizeMotors() {
    Angle armAngle = Degrees.of(encoder.get());
    leftPivotEncoder.setPosition(armAngle.in(Rotations));
    rightPivotEncoder.setPosition(armAngle.in(Rotations));
  }


  @Override
  public void periodic() {
    // subsystemTracker.updateOuttakeRealPosition(outtakePose, getMeasurement());
    
    // switch (subsystemTracker.getElevatorPose()) {
    // case INITAL:
    // if (subsystemTracker.getElevatorRealPosition() < 30.0)
    // setOuttakePosition(OuttakePose.INIT);
    // break;

    // case L2:
    // setOuttakePosition(OuttakePose.MIDL2);
    // break;

    // case L3:
    // setOuttakePosition(OuttakePose.INIT);

    // case L4:
    // setOuttakePosition(OuttakePose.DEPOSIT);
    // break;

    // default:
    // break;
    // }

    // double position = getMeasurement();
    // double velocity = pid.getSetpoint().velocity;

    // double pidOutput = pid.calculate(position);
    // double feedforwardOutput = feedforward.calculate(position, velocity);
    // double output = pidOutput + feedforwardOutput;

    // output = MathUtil.clamp(output, -0.18, 0.09);

    // SmartDashboard.putNumber("O. Current Pos Radians", position);
    // SmartDashboard.putNumber("O. Current Pos Degrees", Math.toDegrees(position));
    // SmartDashboard.putNumber("O. PID Output", pidOutput);
    // SmartDashboard.putNumber("O. Output", output);
    // SmartDashboard.putNumber("O. Setpoint Radians", pid.getSetpoint().position);
    // SmartDashboard.putNumber("O. Setpoint Degrees",
    // Math.toDegrees(pid.getSetpoint().position));
    // SmartDashboard.putNumber("O. Left Power", leftPivotMotor.get());
    // SmartDashboard.putNumber("O. Right Power", rightPivotMotor.get());

    // leftPivotMotor.set(output);
    // rightPivotMotor.set(output);

    SmartDashboard.putNumber("Arm", getMeasurement());
  }

  // Função para obter a medição da posição (encoder)
  public double getMeasurement() {
    return Rotations.of(leftPivotEncoder.getPosition()).in(Degree);
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
