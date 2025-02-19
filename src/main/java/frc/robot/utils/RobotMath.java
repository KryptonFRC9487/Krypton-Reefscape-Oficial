package frc.robot.utils;

public class RobotMath {
  public static double normalize360RangeTo180Range(double rawAngle) {
    return (rawAngle > 180) ? rawAngle - 360 : rawAngle;
  }
}
