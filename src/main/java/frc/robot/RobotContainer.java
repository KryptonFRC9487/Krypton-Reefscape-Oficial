// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {

  private XboxController controller;
  private ElevatorSubsystem elevatorSubsystem;

  public RobotContainer() {
    controller = new XboxController(0);
    elevatorSubsystem = new ElevatorSubsystem();

    configureBindings();
  }

    private void configureBindings() {
      new JoystickButton(controller, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.setTarget(1)));

      new JoystickButton(controller, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.setTarget(17)));
    }

  public Command getAutonomousCommand() {
    return null;
  }
}
