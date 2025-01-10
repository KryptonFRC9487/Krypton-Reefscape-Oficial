package frc.lib;

public interface IMotorController {
  public void setPower(double power);

  public void setInverted(boolean isInverted);
  
  public void setBrake(boolean isBrake);
}
