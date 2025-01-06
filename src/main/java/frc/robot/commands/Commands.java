package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Buttons;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Subsystem;

public class Commands extends Command{

    private final XboxController pController;
    private final Subsystem subsystem;

public Commands(
    XboxController pController,
    Subsystem Subsystem
) {
    this.pController = pController,
    this.subsystem = Subsystem
 }

 @Override
 public void execute(){
    if(pController.getRawButton(Buttons.BUTTON_A)){
        Subsystem.TestSpeed();
    }
 }
}

