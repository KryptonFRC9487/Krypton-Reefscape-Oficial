package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.utils.SubsystemTracker;

public class SysIdRoutineBot {
  private final SubsystemTracker m_subsystemTracker = new SubsystemTracker();
  private final OuttakeSubsystem m_outtake = new OuttakeSubsystem(m_subsystemTracker);

  CommandXboxController m_driverController = new CommandXboxController(Constants.GamepadConstants.P2_PORT);

  public SysIdRoutineBot() {
    configureBindings();
  }

  public void configureBindings() {
    m_driverController
        .a()
        .and(m_driverController.rightBumper())
        .whileTrue(m_outtake.runSysIdRoutine());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
