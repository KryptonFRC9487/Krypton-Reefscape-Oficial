package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;

public class OuttakeCommand extends Command{

   private final XboxController p2Controller;
   private OuttakeSubsystem outtakeSubsystem;

   public OuttakeCommand(OuttakeSubsystem outtakeSubsystem, XboxController p2Controller) {
      this.p2Controller = p2Controller;
      this.outtakeSubsystem = outtakeSubsystem;

      addRequirements(outtakeSubsystem);
   }
}

