package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.OuttakeConstants.ArmConfig.kMinSafeAngle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReefsConstants.ReefsScorePose;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OuttakePivotSubsystem;

public class ScoreSystem {
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final OuttakePivotSubsystem m_outtakeSubsystem;
  private ReefsScorePose m_reefsScorePose;

  public ScoreSystem(ElevatorSubsystem elevatorSubsystem, OuttakePivotSubsystem outtakeSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_outtakeSubsystem = outtakeSubsystem;

    setInitialPose();
  }

  public Command scoreCoral(ReefsScorePose reefsScorePose) {
    setReefsTarget(reefsScorePose);

    if (m_reefsScorePose == ReefsScorePose.INITAL || m_reefsScorePose == ReefsScorePose.L2) {
      return m_outtakeSubsystem.setOuttakePositionCmd(kMinSafeAngle.in(Degrees))
          .until(() -> m_outtakeSubsystem.outtakeIsSafe())
          .andThen(m_elevatorSubsystem.setElevatorPoseCmd(m_reefsScorePose)
              .until(() -> m_elevatorSubsystem.getElevatorPosition() <= 30.0)) // 30.0 é a altura safe para o outtake
          .andThen(m_outtakeSubsystem.setOuttakePositionCmd(m_reefsScorePose));
    }

    return m_elevatorSubsystem.setElevatorPoseCmd(m_reefsScorePose)
        .alongWith(m_outtakeSubsystem.setOuttakePositionCmd(reefsScorePose));
  }

  public void setReefsTarget(ReefsScorePose reefsScorePose) {
    m_reefsScorePose = reefsScorePose;
  }

  public ReefsScorePose getReefsTarget() {
    return m_reefsScorePose;
  }

  public void updateTelemetry() {
    SmartDashboard.putNumber("Reefs Target Angle", getReefsTarget().angle);
    SmartDashboard.putNumber("Reefs Target Height", getReefsTarget().height);
  }

  private void setInitialPose() {
    setReefsTarget(ReefsScorePose.INITAL);

    m_outtakeSubsystem.setOuttakePose(m_reefsScorePose);
    m_elevatorSubsystem.setElevatorPose(m_reefsScorePose);
  }
}
