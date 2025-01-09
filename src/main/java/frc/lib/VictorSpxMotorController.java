package frc.lib;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class VictorSpxMotorController implements IMotorController {

  private WPI_VictorSPX motor;

  public VictorSpxMotorController(int deviceId) {
    motor = new WPI_VictorSPX(deviceId);
  }

  @Override
  public void setPower(double power) {
    motor.set(power);
  }

  @Override
  public void setInverted(boolean isInverted) {
    motor.setInverted(isInverted);
  }
}
