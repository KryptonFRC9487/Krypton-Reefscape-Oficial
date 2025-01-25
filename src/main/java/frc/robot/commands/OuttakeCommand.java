package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.Constants.OuttakeConstants.OuttakePose;

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
    if (p2Controller.getXButtonPressed()) {
      outtakeSubsystem.setOuttakePosition(OuttakePose.INIT.value);
    }

    if (p2Controller.getBButtonPressed()) {
      outtakeSubsystem.setOuttakePosition(OuttakePose.DEPOSIT.value);
    }

    if (p2Controller.getRightTriggerAxis() != 0) {
      if (!OuttakeSubsystem.limitSwitch) {
        outtakeSubsystem.setOuttakeSpeed(0);
      } else {
        outtakeSubsystem.setOuttakeSpeed(0.5);
      }
    } else if (p2Controller.getLeftTriggerAxis() != 0) {
      outtakeSubsystem.setOuttakeSpeed(-0.5);
    } else {
      outtakeSubsystem.setOuttakeSpeed(0);
    }
  }
}
