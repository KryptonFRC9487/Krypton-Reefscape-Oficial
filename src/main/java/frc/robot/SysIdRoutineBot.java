package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.OuttakePivotSubsystem;

public class SysIdRoutineBot {
  private final OuttakePivotSubsystem m_outtake = new OuttakePivotSubsystem();

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
