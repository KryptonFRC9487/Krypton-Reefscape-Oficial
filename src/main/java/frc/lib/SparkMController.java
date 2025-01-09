package frc.lib;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SparkMController implements IMotorController {

  private SparkMax motor;
  private SparkMaxConfig config;

  public SparkMController(int deviceId) {
    motor = new SparkMax(deviceId, MotorType.kBrushless);
    config = new SparkMaxConfig();

    updateConfig();
  }

  @Override
  public void setPower(double power) {
    motor.set(power);
  }

  @Override
  public void setInverted(boolean isInverted) {
    config.inverted(isInverted);
    updateConfig();
  }


  @Override
  public void setBrake(boolean isBrake) {
    SparkBaseConfig.IdleMode mode = isBrake ? IdleMode.kBrake : IdleMode.kCoast;

    config.idleMode(mode);
    updateConfig();
  }

  private void updateConfig() {
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
