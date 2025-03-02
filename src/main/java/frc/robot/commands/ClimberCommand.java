package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Buttons;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command{

    private final XboxController p1Controller;
    private ClimberSubsystem climberSubsystem;

public ClimberCommand(
    ClimberSubsystem climberSubsystem,
    XboxController p2Controller
){
    this.p1Controller = p2Controller;
    this.climberSubsystem = climberSubsystem;

    addRequirements(climberSubsystem);
    
}

@Override
public void execute(){

    if(p1Controller.getRawButton(Buttons.BUTTON_X)){
        climberSubsystem.setClimberSpeed(0.5);
    } else if(p1Controller.getRawButton(Buttons.BUTTON_Y)){
        climberSubsystem.setClimberSpeed(-0.5);
    } else{
        climberSubsystem.setClimberSpeed(0);
    }


}
    
}