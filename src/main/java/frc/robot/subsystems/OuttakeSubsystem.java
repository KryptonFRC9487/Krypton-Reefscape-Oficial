package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;

// import frc.lib.SparkMController
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeSubsystem extends SubsystemBase{

    public SparkMax anglemotor, collectmotor;
    private final PIDController pid;
    public DutyCycleEncoder througbore1;
    public final DigitalInput m_limitSwitch;

    public static boolean limitSwitch;

    public OuttakeSubsystem(){

        anglemotor =
            new SparkMax(OuttakeConstants.ANGLE_ID, MotorType.kBrushless);

        SparkMaxConfig angleMotorConfig = new SparkMaxConfig();

        angleMotorConfig.idleMode(IdleMode.kBrake);

        anglemotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        collectmotor = 
            new SparkMax(OuttakeConstants.COLLECT_ID, MotorType.kBrushless);

        m_limitSwitch = new DigitalInput(3);


        pid = new PIDController(0, 0, 0);
        througbore1 = new DutyCycleEncoder(0);

    }

    @Override
    public void periodic(){

        double output = pid.calculate(getMeasurement());
        output = MathUtil.clamp(output, -0.8,0.8); 
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
