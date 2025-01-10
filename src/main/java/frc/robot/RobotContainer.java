// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.Constants.Controle;

public class RobotContainer {

  // private XboxController p2Controller;
  private ElevatorSubsystem elevatorSubsystem;
  private OuttakeSubsystem outtakeSubsystem;

  private XboxController p1Controller = new XboxController(Controle.P1PORT);
  private XboxController p2Controller = new XboxController(Controle.P2PORT);


  public RobotContainer() {

    elevatorSubsystem = new ElevatorSubsystem();
    outtakeSubsystem = new OuttakeSubsystem();


    configureBindings();
  }

    private void configureBindings() {
      new JoystickButton(p2Controller, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.setTarget(1)));

      new JoystickButton(p2Controller, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.setTarget(17)));
    }

  public Command getAutonomousCommand() {
    return null;
  }
}
