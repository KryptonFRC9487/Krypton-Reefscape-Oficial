package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Buttons;
import frc.robot.subsystems.AlgaePivotSubsystem;

public class AlgaePivotCommand extends Command {

  private final XboxController p2Controller;
  private AlgaePivotSubsystem algaeSubsystem;

  public AlgaePivotCommand(
      AlgaePivotSubsystem algaesubsystem,
      XboxController p2Controller) {

    this.p2Controller = p2Controller;
    this.algaeSubsystem = algaesubsystem;

    addRequirements(algaesubsystem);
  }

  @Override
  public void execute() {

    // if(p2Controller.getRawButton(Buttons.BUTTON_Y)){
    // algaeSubsystem.setKrakenSpeed(0.2);
    // } else{
    // algaeSubsystem.setKrakenSpeed(0);
    // }

    if (p2Controller.getRawButton(Buttons.BUTTON_Y)) {
      algaeSubsystem.setKrakenPosition(15);
    } else if (p2Controller.getRawButton(Buttons.BUTTON_A)) {
      algaeSubsystem.setKrakenPosition(0);
    }
  }

}
