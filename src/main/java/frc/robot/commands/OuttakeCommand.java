package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.Outtake;
import frc.robot.Constants.Outtake.OuttakePose;

public class OuttakeCommand extends Command{

   private final XboxController p2Controller;
   private OuttakeSubsystem outtakeSubsystem;

   private OuttakePose outtakePose;

   public OuttakeCommand( 
   OuttakeSubsystem outtakeSubsystem,
   XboxController p2Controller
   ) {

      this.p2Controller = p2Controller;
      this.outtakeSubsystem = outtakeSubsystem;

      addRequirements(outtakeSubsystem);
   }

   @Override
   public void initialize(){
      outtakePose = OuttakePose.INIT;
   }

   @Override
   public void execute(){

      
      if(p2Controller.getRawButton(Buttons.BUTTON_X)){
         outtakePose = OuttakePose.DEPOSIT;
      } else if(p2Controller.getRawButton(Buttons.BUTTON_B)){
         outtakePose = OuttakePose.INIT;
      }
      
      // if(p2Controller.getRightTriggerAxis() != 0){
      //    outtakeSubsystem.setOuttakeSpeed(0);
      // } else if(p2Controller.getLeftTriggerAxis() != 0){
      //    outtakeSubsystem.setOuttakeSpeed(-0);
      // } else{
      //    outtakeSubsystem.setOuttakeSpeed(0);
      // }

      if(p2Controller.getRightTriggerAxis() != 0){
         if(!OuttakeSubsystem.limitSwitch){
             outtakeSubsystem.setOuttakeSpeed(0);
         } else {
             outtakeSubsystem.setOuttakeSpeed(0.5);
         }
     } else if(p2Controller.getLeftTriggerAxis() != 0){
         outtakeSubsystem.setOuttakeSpeed(-0.5);
     } else {
         outtakeSubsystem.setOuttakeSpeed(0);
     } 

      switch (outtakePose) {
         case INIT:
           outtakeSubsystem.setOuttakePosition(Outtake.OUTTAKE_INIT);
           break;
         case DEPOSIT:
           outtakeSubsystem.setOuttakePosition(Outtake.OUTTAKE_DEPOSIT);
           break;
       }

   }
}

