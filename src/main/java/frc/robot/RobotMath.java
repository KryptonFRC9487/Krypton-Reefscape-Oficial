package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.OuttakeConstants.ArmConfig;

public class RobotMath {
  public static class Arm {

    public static Angle convertSensorUnitsToAngle(Angle measurement) {
      return Degrees.of(measurement.in(Rotations) / ArmConfig.kReduction);
    }
  }
}
