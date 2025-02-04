package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.Constants.Buttons;

public class OuttakeCommand extends Command {

  private final XboxController p2Controller;
  private OuttakeSubsystem outtakeSubsystem;

  public OuttakeCommand(
      OuttakeSubsystem outtakeSubsystem,
      XboxController p2Controller) {

    this.p2Controller = p2Controller;
    this.outtakeSubsystem = outtakeSubsystem;

    addRequirements(outtakeSubsystem);
  }

  @Override
  public void execute() {
    if (p2Controller.getRightTriggerAxis() != 0) {
      if (!outtakeSubsystem.outtakeHasCoral()) {
        outtakeSubsystem.setOuttakeSpeed(0);
      } else {
        outtakeSubsystem.setOuttakeSpeed(-0.35);
      }
    } else if (p2Controller.getLeftTriggerAxis() != 0) {
      outtakeSubsystem.setOuttakeSpeed(0.35);
    } else {
      outtakeSubsystem.setOuttakeSpeed(0);
    }

    if(p2Controller.getRawButton(Buttons.LEFT_BUMPER)){
      outtakeSubsystem.setOuttakeSpeed(-0.3);
    }    
  }
}
