package frc.lib;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMotorController implements IMotorController {

  private SparkMax motor;

  public SparkMotorController(int deviceId) {
    motor = new SparkMax(deviceId, MotorType.kBrushless);
  }

  @Override
  public void setPower(double power) {
    motor.set(power);
  }

  @Override
  public void setInverted(boolean isInverted) {
    motor.configure(new SparkMaxConfig().inverted(isInverted), ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }
}
