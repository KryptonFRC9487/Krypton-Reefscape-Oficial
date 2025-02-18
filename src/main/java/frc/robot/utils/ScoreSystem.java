package frc.robot.utils;

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
  }

  public Command scoreCoral(ReefsScorePose reefsScorePose) {
    setReefsTarget(reefsScorePose);

    if (m_reefsScorePose == ReefsScorePose.INITAL) {
      return m_outtakeSubsystem.setOuttakePositionCmd(ReefsScorePose.L4)
          .until(() -> m_outtakeSubsystem.getOuttakePosition() >= ReefsScorePose.L4.angle)
          .andThen(m_elevatorSubsystem.setElevatorPoseCmd(ReefsScorePose.INITAL))
          .until(() -> m_elevatorSubsystem.getElevatorPosition() <= 30.0) // 30.0 é a altura safe para o outtake descer
          .andThen(m_outtakeSubsystem.setOuttakePositionCmd(ReefsScorePose.INITAL));
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
}
