package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.ClimberPositions.ClimberPose;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command{

    private final XboxController p1Controller;
    private ClimberSubsystem climberSubsystem;

    private ClimberPose climberPose;

public ClimberCommand(
    ClimberSubsystem climberSubsystem,
    XboxController p1Controller
){
    this.p1Controller = p1Controller;
    this.climberSubsystem = climberSubsystem;

    addRequirements(climberSubsystem);
    
}

// @Override
// public void initialize(){
//     climberPose = ClimberPose.UP;
// }

@Override
public void execute(){

    if(p1Controller.getRawButton(Buttons.BUTTON_A)){
        climberSubsystem.setClimberSpeed(-2);
    } else if(p1Controller.getRawButton(Buttons.BUTTON_B)){
        climberSubsystem.setClimberSpeed(2);
    } else{
        climberSubsystem.setClimberSpeed(0);
    }

    // if(p2Controller.getRawButton(Buttons.BUTTON_A)){ 
    //     climberPose = ClimberPose.UP;
    // } else if(p2Controller.getRawButton(Buttons.BUTTON_B)){
    //     climberPose = ClimberPose.DOWN;
    // }

    // switch (climberPose) {
    //     case UP:
    //       climberSubsystem.setClimberPosition(ClimberPositions.CLIMBER_UP);
    //       break;
    //     case DOWN:
    //       climberSubsystem.setClimberPosition(ClimberPositions.CLIMBER_DOWM);
    //       break;
    //   }  


}
    
}
