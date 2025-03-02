package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.OuttakeConstants.ArmConfig.kMinSafeAngle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReefsConstants.ReefsScorePose;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OuttakePivotSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class ScoreSystem {
  private final ElevatorSubsystem m_elevatorSubsystem;
  // private final OuttakeSubsystem m_outtakeSubsystem;
  // private final OuttakePivotSubsystem m_outtakePivotSubsystem;
  private ReefsScorePose m_reefsScorePose;
  private boolean m_coralSystemInSafeMode;

  public ScoreSystem(ElevatorSubsystem elevatorSubsystem) 
  {
    m_elevatorSubsystem = elevatorSubsystem;  

    setInitialPose();
  }

  // public ScoreSystem(
  //     ElevatorSubsystem elevatorSubsystem,
  //     OuttakePivotSubsystem outtakePivotSubsystem,
  //     OuttakeSubsystem outtakeSubsystem) {

  //   m_elevatorSubsystem = elevatorSubsystem;
  //   m_outtakePivotSubsystem = outtakePivotSubsystem;
  //   m_outtakeSubsystem = outtakeSubsystem;

  //   setInitialPose();
  // }

  public Command scoreCoral(ReefsScorePose reefsScorePose) {
    setCoralSystemSafeMode(reefsScorePose);
    setReefsTarget(reefsScorePose);

    return m_elevatorSubsystem.setElevatorPoseCmd(reefsScorePose);
    // if (m_coralSystemInSafeMode) {
    //   return m_outtakePivotSubsystem.setOuttakePositionCmd(kMinSafeAngle.in(Degrees))
    //       .until(() -> m_outtakePivotSubsystem.outtakeIsSafe())
    //       .andThen(m_elevatorSubsystem.setElevatorPoseCmd(m_reefsScorePose)
    //           .until(() -> m_elevatorSubsystem.getElevatorPosition() <= 10.0))
    //       .andThen(m_outtakePivotSubsystem.setOuttakePositionCmd(m_reefsScorePose));
    // }

    // return m_outtakePivotSubsystem.setOuttakePositionCmd(m_reefsScorePose)
    //     .alongWith(m_elevatorSubsystem.setElevatorPoseCmd(m_reefsScorePose));
  }

  public Command scoreCoralAuto(ReefsScorePose reefsScorePose) {
    setCoralSystemSafeMode(reefsScorePose);
    setReefsTarget(reefsScorePose);

    return null;

    // return m_elevatorSubsystem.setElevatorPoseCmd(m_reefsScorePose)
    //     .alongWith(m_outtakePivotSubsystem.setOuttakePositionCmd(m_reefsScorePose))
    //     .until(() -> scoringPoseReached())
    //     .andThen(m_outtakeSubsystem.setOuttakeSpeedCmd(0.0).withTimeout(0.5))
    //     .andThen(m_outtakePivotSubsystem.setOuttakePositionCmd(kMinSafeAngle.in(Degrees))
    //         .until(() -> m_outtakePivotSubsystem.outtakeIsSafe()))
    //     .andThen(m_elevatorSubsystem.setElevatorPoseCmd(ReefsScorePose.INITAL)
    //         .until(() -> m_elevatorSubsystem.getElevatorPosition() <= 10.0)) // 30.0 é a altura safe para o outtake
    //     .andThen(m_outtakePivotSubsystem.setOuttakePositionCmd(ReefsScorePose.INITAL));
  }

  // private boolean scoringPoseReached() {
  //   return m_elevatorSubsystem.atPose(m_reefsScorePose)
  //       && m_outtakePivotSubsystem.atPose(m_reefsScorePose);
  // }

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

    // m_outtakePivotSubsystem.setOuttakePose(m_reefsScorePose);
    m_elevatorSubsystem.setElevatorPose(m_reefsScorePose);
  }
}
