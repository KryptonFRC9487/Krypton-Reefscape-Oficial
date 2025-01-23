package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeSubsystem extends SubsystemBase{

    public SparkMax anglemotor, anglemotor2, collectmotor;
    private final PIDController pid;
    public DutyCycleEncoder througbore1;
    public final DigitalInput m_limitSwitch;

    public static boolean limitSwitch;

    public OuttakeSubsystem(){

        anglemotor = new SparkMax(OuttakeConstants.ANGLE_ID, MotorType.kBrushless);
        anglemotor2 = new SparkMax(OuttakeConstants.ANGLE2_ID, MotorType.kBrushless);  

        SparkMaxConfig angleMotorConfig = new SparkMaxConfig();
        SparkMaxConfig angleMotorConfig2 = new SparkMaxConfig();

        angleMotorConfig.idleMode(IdleMode.kBrake);
        angleMotorConfig2.follow(anglemotor, true);

        anglemotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        anglemotor2.configure(angleMotorConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        collectmotor = 
            new SparkMax(OuttakeConstants.COLLECT_ID, MotorType.kBrushless);

        m_limitSwitch = new DigitalInput(3);

        pid = new PIDController(1.35, 0, 0);
        througbore1 = new DutyCycleEncoder(0);

    }

    @Override
    public void periodic(){
        double output = pid.calculate(getMeasurement());
        output = MathUtil.clamp(output, -0.5,0.1); 
        anglemotor.set(output);
        
        SmartDashboard.putNumber("Encoder Outtake",getMeasurement());
    }

    public double getMeasurement(){
        return througbore1.get();
        }
        

    public void setOuttakePosition(double setpoint) {
        pid.setSetpoint(setpoint);
    }
    
    public void setOuttakeSpeed(double speed){
        collectmotor.set(speed);
    }
    

    
}
