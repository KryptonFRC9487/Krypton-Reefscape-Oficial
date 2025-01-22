package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final SparkMax leftMotor, rightMotor;
  private final SparkMaxConfig leftMotorConfig, rightMotorConfig;
  private final SparkClosedLoopController leftClosedLoopController;
  private final RelativeEncoder leftEncoder, rightEncoder;

  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
      ElevatorConstants.kS,
      ElevatorConstants.kG,
      ElevatorConstants.kV,
      ElevatorConstants.kA);

  public ElevatorSubsystem() {
    leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    leftMotorConfig = new SparkMaxConfig();
    rightMotorConfig = new SparkMaxConfig();

    leftClosedLoopController = leftMotor.getClosedLoopController();

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftMotorConfig
        .smartCurrentLimit(80)
        .closedLoopRampRate(0.25).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
        .outputRange(-1.0, 1.0);

    leftMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    rightMotorConfig
        .follow(leftMotor, true)
        .smartCurrentLimit(80);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setTarget(double target) {
    leftClosedLoopController.setReference(
        target,
        ControlType.kPosition);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Elevator Position (Rotations)", leftEncoder.getPosition());
    SmartDashboard.putNumber("Left Elevator Velocity (Rotations per Second)", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Left Elevator Voltage", leftMotor.getBusVoltage());

    SmartDashboard.putNumber("Right Elevator Position (Rotations)", rightEncoder.getPosition());
    SmartDashboard.putNumber("Right Elevator Velocity (Rotations per Second)", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Right Elevator Voltage", rightMotor.getBusVoltage());
  }

}
