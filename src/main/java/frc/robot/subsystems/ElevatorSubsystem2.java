package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem2 extends SubsystemBase {

  private final SparkMax leftMotor, rightMotor;
  private final SparkMaxConfig leftMotorConfig, rightMotorConfig;
  private final RelativeEncoder leftEncoder, rightEncoder;

  private final ProfiledPIDController pidController = new ProfiledPIDController(
      ElevatorConstants.kP,
      ElevatorConstants.kI,
      ElevatorConstants.kD,
      ElevatorConstants.MOVEMENT_CONSTRAINTS);

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
      ElevatorConstants.kS,
      ElevatorConstants.kG,
      ElevatorConstants.kV,
      ElevatorConstants.kA);

  public ElevatorSubsystem2() {
    leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    leftMotorConfig = new SparkMaxConfig();
    rightMotorConfig = new SparkMaxConfig();

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    rightMotorConfig.follow(leftMotor, true);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setVoltage(double voltage) {
    leftMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  public Command moveToCurrentGoalCommand() {
    return run(() -> {
      double feedbackVoltage = pidController.calculate(leftEncoder.getPosition());
      double feedforwardVoltage = feedforward.calculate(pidController.getSetpoint().velocity);
      setVoltage(feedbackVoltage + feedforwardVoltage);
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("L. Elevator Position (Rotations)", leftEncoder.getPosition());
    SmartDashboard.putNumber("L. Elevator Velocity (Rotations per Second)", leftEncoder.getVelocity());
    SmartDashboard.putNumber("L. Elevator Applied Voltage", leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
    SmartDashboard.putNumber("R. Elevator Applied Output", leftMotor.getAppliedOutput());

    SmartDashboard.putNumber("R. Elevator Position (Rotations)", rightEncoder.getPosition());
    SmartDashboard.putNumber("R. Elevator Velocity (Rotations per Second)", rightEncoder.getVelocity());
    SmartDashboard.putNumber("R. Elevator Applied Voltage", rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());
    SmartDashboard.putNumber("R. Elevator Applied Output", rightMotor.getAppliedOutput());
  }

}
