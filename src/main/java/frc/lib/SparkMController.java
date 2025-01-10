package frc.lib;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SparkMController implements IMotorController {

  public SparkMax controller;
  public SparkMaxConfig config;

  public SparkMController(int deviceId) {
    controller = new SparkMax(deviceId, MotorType.kBrushless);
    config = new SparkMaxConfig();

    updateConfig();
  }

  @Override
  public void setPower(double power) {
    controller.set(power);
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

  public void follow(SparkMController leader) {
    config.follow(leader.getController());
    updateConfig();
  }

  public void updateConfig() {
    controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public SparkMax getController() {
    return controller;
  }
}
