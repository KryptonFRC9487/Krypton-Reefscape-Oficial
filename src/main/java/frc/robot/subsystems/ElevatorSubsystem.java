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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax leftMotor, rightMotor;
    private SparkMaxConfig leftMotorConfig, rightMotorConfig;
    private SparkClosedLoopController leftClosedLoopController, rightClosedLoopController;
    private RelativeEncoder leftEncoder, rightEncoder;

    public ElevatorSubsystem(){
        leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

        leftMotorConfig = new SparkMaxConfig();
        rightMotorConfig = new SparkMaxConfig();

        leftClosedLoopController = leftMotor.getClosedLoopController();
        rightClosedLoopController = rightMotor.getClosedLoopController();

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftMotorConfig.inverted(true);

        InitializePidControl(leftMotorConfig);
        InitializePidControl(rightMotorConfig);

        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void InitializePidControl(SparkMaxConfig config) {
        config.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(0.05, 0.0, 0, 0)
            .outputRange(-1, 1);
    }

    public void setTarget(double target) {
        leftClosedLoopController.setReference(target, ControlType.kPosition);
        rightClosedLoopController.setReference(target, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Elevator Position", leftEncoder.getPosition());
        SmartDashboard.putNumber("Left Elevator Velocity", leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right Elevator Position", rightEncoder.getPosition());
        SmartDashboard.putNumber("Right Elevator Velocity", rightEncoder.getVelocity());
    }
}
