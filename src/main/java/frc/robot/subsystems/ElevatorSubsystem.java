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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ReefsConstants.ReefsScorePose;

public class ElevatorSubsystem extends SubsystemBase {

  private final SparkMax leftMotor, rightMotor;
  private final SparkMaxConfig leftMotorConfig, rightMotorConfig;
  private final SparkClosedLoopController leftClosedLoopController;
  private final RelativeEncoder leftEncoder, rightEncoder;

  public ElevatorSubsystem() {
    leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    leftMotorConfig = new SparkMaxConfig();
    rightMotorConfig = new SparkMaxConfig();

    leftClosedLoopController = leftMotor.getClosedLoopController();

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftMotorConfig
        .closedLoopRampRate(0.1).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
          ElevatorConstants.kP,
          ElevatorConstants.kI,
          ElevatorConstants.kD,
          ElevatorConstants.kFF 
        )
        .velocityFF(ElevatorConstants.kVelocityFF)
        // .outputRange(-1.0, 1.0);
        .outputRange(-0.8, 0.8);

    leftMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    rightMotorConfig.follow(leftMotor, true);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setElevatorPose(ReefsScorePose reefsScorePose) {
    leftClosedLoopController.setReference(
        reefsScorePose.height,
        ControlType.kPosition);
  }

  public Command setElevatorPoseCmd(ReefsScorePose reefsScorePose) {
    return run(() -> setElevatorPose(reefsScorePose));
  }

  public double getElevatorPosition() {
    return leftEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("L. Elevator Position (Rotations)", leftEncoder.getPosition());
    SmartDashboard.putNumber("L. Elevator Velocity (Rotations per Second)", leftEncoder.getVelocity());
    SmartDashboard.putNumber("L. Elevator Applied Voltage", leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
    SmartDashboard.putNumber("L. Elevator Applied Output", leftMotor.getAppliedOutput());

    SmartDashboard.putNumber("R. Elevator Position (Rotations)", rightEncoder.getPosition());
    SmartDashboard.putNumber("R. Elevator Velocity (Rotations per Second)", rightEncoder.getVelocity());
    SmartDashboard.putNumber("R. Elevator Applied Voltage", rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());
    SmartDashboard.putNumber("R. Elevator Applied Output", rightMotor.getAppliedOutput());
  }
}