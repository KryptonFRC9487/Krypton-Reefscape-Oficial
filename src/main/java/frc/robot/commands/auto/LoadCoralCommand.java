package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;

public class LoadCoralCommand extends Command {
  private final OuttakeSubsystem m_outtakeSubsystem;

  public LoadCoralCommand(OuttakeSubsystem subsystem) {
    m_outtakeSubsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    if (m_outtakeSubsystem.outtakeHasCoral()) {
      m_outtakeSubsystem.setOuttakeSpeed(0.0);
      this.cancel();
    } else {
      m_outtakeSubsystem.setOuttakeSpeed(-0.27);
    }
  }

  @Override
  public boolean isFinished() {
    return m_outtakeSubsystem.outtakeHasCoral();
  }
}
