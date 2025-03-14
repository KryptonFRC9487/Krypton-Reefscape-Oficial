package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants.Gains;
import frc.robot.Constants.ElevatorConstants.HardwareConfig;
import frc.robot.Constants.ReefsConstants.ReefsScorePose;

public class ElevatorSubsystem extends SubsystemBase {

  private final SparkMax leftMotor, rightMotor;
  private final SparkMaxConfig leftMotorConfig, rightMotorConfig;
  private final SparkClosedLoopController rightClosedLoopController;
  private final RelativeEncoder leftEncoder, rightEncoder;

  public ElevatorSubsystem() {
    leftMotor = new SparkMax(HardwareConfig.kLeftMotorId, MotorType.kBrushless);
    rightMotor = new SparkMax(HardwareConfig.kRightMotorId, MotorType.kBrushless);

    leftMotorConfig = new SparkMaxConfig();
    rightMotorConfig = new SparkMaxConfig();

    rightClosedLoopController = rightMotor.getClosedLoopController();

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    rightMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            Gains.kP,
            Gains.kI,
            Gains.kD,
            Gains.kFF)
        .velocityFF(Gains.kVelocityFF)
        .outputRange(-HardwareConfig.kOutputRange, HardwareConfig.kOutputRange);

    rightMotorConfig
        .smartCurrentLimit(HardwareConfig.kStallCurrentLimit)
        .closedLoopRampRate(HardwareConfig.kClosedLoopRate).encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    leftMotorConfig
        .smartCurrentLimit(HardwareConfig.kStallCurrentLimit)
        .closedLoopRampRate(HardwareConfig.kClosedLoopRate).encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    leftMotorConfig.follow(rightMotor, true);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setElevatorPose(ReefsScorePose reefsScorePose) {
    rightClosedLoopController.setReference(
        reefsScorePose.height,
        ControlType.kPosition);
  }

  public Command setElevatorPoseCmd(ReefsScorePose reefsScorePose) {
    return run(() -> setElevatorPose(reefsScorePose));
  }

  public double getElevatorPosition() {
    return rightEncoder.getPosition();
  }

  public boolean atPose(ReefsScorePose reefsScorePose) {
    return MathUtil.isNear(reefsScorePose.height, getElevatorPosition(), 5.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("L. Elevator Position (Rotations)", leftEncoder.getPosition());
    SmartDashboard.putNumber("L. Elevator Velocity (Rotations per Second)", leftEncoder.getVelocity());
    SmartDashboard.putNumber("L. Elevator Applied Voltage", leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
    SmartDashboard.putNumber("L. Elevator Applied Output", leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("L. Elevator Temperature", leftMotor.getMotorTemperature());

    SmartDashboard.putNumber("R. Elevator Position (Rotations)", rightEncoder.getPosition());
    SmartDashboard.putNumber("R. Elevator Velocity (Rotations per Second)", rightEncoder.getVelocity());
    SmartDashboard.putNumber("R. Elevator Applied Voltage", rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());
    SmartDashboard.putNumber("R. Elevator Applied Output", rightMotor.getAppliedOutput());
    SmartDashboard.putNumber("R. Elevator Temperature", rightMotor.getMotorTemperature());
  }
}