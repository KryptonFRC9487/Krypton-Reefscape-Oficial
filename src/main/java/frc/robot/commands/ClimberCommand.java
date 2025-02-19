package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Buttons;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command{

    private final XboxController p2Controller;
    private ClimberSubsystem climberSubsystem;

public ClimberCommand(
    ClimberSubsystem climberSubsystem,
    XboxController p2Controller
){
    this.p2Controller = p2Controller;
    this.climberSubsystem = climberSubsystem;

    addRequirements(climberSubsystem);
    
}

@Override
public void execute(){


}
    
}
