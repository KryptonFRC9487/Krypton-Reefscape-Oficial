// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ElevatorConstants;

// public class ElevatorSubsystem2 extends SubsystemBase {

//     private SparkMax leftMotor, rightMotor;
//     private SparkMaxConfig leftMotorConfig, rightMotorConfig;
//     private RelativeEncoder leftEncoder, rightEncoder;

//     private PIDController leftPidController, rightPidController;

//     public ElevatorSubsystem2() {
//         leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
//         rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

//         leftMotorConfig = new SparkMaxConfig();
//         rightMotorConfig = new SparkMaxConfig();

//         leftEncoder = leftMotor.getEncoder();
//         rightEncoder = rightMotor.getEncoder();

//         leftMotorConfig.inverted(true);
//         rightMotorConfig.follow(leftMotor);

//         leftPidController = new PIDController(0.3, 0, 0);
//         rightPidController = new PIDController(0.3, 0, 0);
        
//         leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//     }

//     public void setTarget(double target) {
//         leftPidController.setSetpoint(target);
//         rightPidController.setSetpoint(target);
//     }

//     @Override
//     public void periodic() {
//         double leftMotorOutput = leftPidController.calculate(leftEncoder.getPosition());
//         double rightMotorOutput = rightPidController.calculate(rightEncoder.getPosition());

//         leftMotorOutput = MathUtil.clamp(leftMotorOutput, -1.0, 1.0);
//         rightMotorOutput = MathUtil.clamp(rightMotorOutput, -1.0, 1.0);

//         leftMotor.set(leftMotorOutput);
//         rightMotor.set(rightMotorOutput);

//         SmartDashboard.putNumber("Left Elevator Output", leftMotorOutput);
//         SmartDashboard.putNumber("Left Elevator Position", leftEncoder.getPosition());
//         SmartDashboard.putNumber("Left Elevator Velocity", leftEncoder.getVelocity());
//         SmartDashboard.putNumber("Right Elevator Output", rightMotorOutput);
//         SmartDashboard.putNumber("Right Elevator Position", rightEncoder.getPosition());
//         SmartDashboard.putNumber("Right Elevator Velocity", rightEncoder.getVelocity());
//     }
// }