package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.OuttakeConstants.ArmConfig.kMinSafeAngle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ReefsConstants.ReefsScorePose;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OuttakePivotSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class ScoreSystem {
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final OuttakeSubsystem m_outtakeSubsystem;
  private final OuttakePivotSubsystem m_outtakePivotSubsystem;
  private ReefsScorePose m_reefsScorePose;
  private boolean m_coralSystemInSafeMode;

  public ScoreSystem(
      ElevatorSubsystem elevatorSubsystem,
      OuttakePivotSubsystem outtakePivotSubsystem,
      OuttakeSubsystem outtakeSubsystem) {

    m_elevatorSubsystem = elevatorSubsystem;
    m_outtakePivotSubsystem = outtakePivotSubsystem;
    m_outtakeSubsystem = outtakeSubsystem;

    setInitialPose();
  }

  public Command scoreCoral(ReefsScorePose reefsScorePose) {
    setCoralSystemSafeMode(reefsScorePose);
    setReefsTarget(reefsScorePose);

    if (m_coralSystemInSafeMode) {
      // if (m_outtakePivotSubsystem.outtakeIsSafe()) {
      // return m_elevatorSubsystem.setElevatorPoseCmd(m_reefsScorePose)
      // .until(() -> m_elevatorSubsystem.atPose(m_reefsScorePose))
      // .andThen(() ->
      // m_outtakePivotSubsystem.setOuttakePositionCmd(m_reefsScorePose));
      // }

      return m_outtakePivotSubsystem.setOuttakePositionCmd(kMinSafeAngle.in(Degrees))
          .until(() -> m_outtakePivotSubsystem.outtakeIsSafe())
          .andThen(m_elevatorSubsystem.setElevatorPoseCmd(m_reefsScorePose)
              .until(() -> m_elevatorSubsystem.atPose(m_reefsScorePose)
                  || m_elevatorSubsystem.getElevatorPosition() <= ReefsScorePose.INITAL.height))
          .andThen(m_outtakePivotSubsystem.setOuttakePositionCmd(m_reefsScorePose));
    }

    return m_outtakePivotSubsystem.setOuttakePositionCmd(m_reefsScorePose)
        .alongWith(m_elevatorSubsystem.setElevatorPoseCmd(m_reefsScorePose));
  }

  public Command loadCoral() {
    return m_outtakeSubsystem.setOuttakeSpeedCmd(-0.27).until(() -> m_outtakeSubsystem.outtakeHasCoral())
        .andThen(m_outtakeSubsystem.setOuttakeSpeedCmd(0.0));
  }

  public Command scoreL4CoralAuto() {
    setCoralSystemSafeMode(ReefsScorePose.L4);
    setReefsTarget(ReefsScorePose.L4);

    return m_outtakePivotSubsystem.setOuttakePositionCmd(ReefsScorePose.L4)
        .andThen(m_elevatorSubsystem.setElevatorPoseCmd(ReefsScorePose.L4)
            .until(() -> m_elevatorSubsystem.atPose(ReefsScorePose.L4)))
        .andThen(() -> new WaitCommand(1.0))
        .andThen(m_outtakeSubsystem.setOuttakeSpeedCmd(0.3))
        .andThen(() -> new WaitCommand(0.5))
        .andThen(m_outtakeSubsystem.setOuttakeSpeedCmd(0.0));
  }

  private boolean scoringPoseReached() {
    return m_elevatorSubsystem.atPose(m_reefsScorePose)
        && m_outtakePivotSubsystem.atPose(m_reefsScorePose);
  }

  public void setReefsTarget(ReefsScorePose reefsScorePose) {
    m_reefsScorePose = reefsScorePose;
  }

  public void setCoralSystemSafeMode(ReefsScorePose reefsScorePose) {
    m_coralSystemInSafeMode = !m_elevatorSubsystem.atPose(ReefsScorePose.INITAL)
        && reefsScorePose == ReefsScorePose.INITAL;
  }

  public ReefsScorePose getReefsTarget() {
    return m_reefsScorePose;
  }

  private void setInitialPose() {
    setReefsTarget(ReefsScorePose.INITAL);

    m_outtakePivotSubsystem.setOuttakePose(m_reefsScorePose);
    m_elevatorSubsystem.setElevatorPose(m_reefsScorePose);
  }
}
