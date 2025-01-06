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
    Subsystem subsystem
) {
    this.pController = pController;
    this.subsystem = subsystem;
 }

 @Override
 public void execute(){
    if(pController.getRawButton(Buttons.BUTTON_A)){
        subsystem.TestSpeed(0.1);   
    }
 }
}

