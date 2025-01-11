// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.Controle;
import frc.robot.commands.SwerveCommand;

public class RobotContainer {

  // private XboxController p2Controller;
  private ElevatorSubsystem elevatorSubsystem;
  private OuttakeSubsystem outtakeSubsystem;

  private XboxController p1Controller = new XboxController(Controle.P1PORT);
  private XboxController p2Controller = new XboxController(Controle.P2PORT);

   private SwerveSubsystem swerve = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );


  public RobotContainer() {

    elevatorSubsystem = new ElevatorSubsystem();
    // outtakeSubsystem = new OuttakeSubsystem(); 


    setDefaultCommands();
    registerAutoCommands();
    configureBindings();
  }

    private void setDefaultCommands() {
     if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      swerve.setDefaultCommand(
      new SwerveCommand(
        swerve,
        () ->
          -MathUtil.applyDeadband(p1Controller.getLeftY(), Controle.DEADBAND),
        () ->
          -MathUtil.applyDeadband(p1Controller.getLeftX(), Controle.DEADBAND),
        () ->
          -MathUtil.applyDeadband(p1Controller.getRightX(), Controle.DEADBAND),
        () -> p1Controller.getRightBumperPressed()
      )
    );
    } else {
      swerve.setDefaultCommand(
      new SwerveCommand(
        swerve,
        () ->
          MathUtil.applyDeadband(p1Controller.getLeftY(), Controle.DEADBAND),
        () ->
          MathUtil.applyDeadband(p1Controller.getLeftX(), Controle.DEADBAND),
        () ->
          MathUtil.applyDeadband(p1Controller.getRightX(), Controle.DEADBAND),
        () -> p1Controller.getRightBumperPressed()
      )
    );
    }
  }

  private void registerAutoCommands() {

  }

  //Heading Correction 
  public void setHeadingCorrection(boolean setHeadingCorrection){
    swerve.swerveDrive.setHeadingCorrection(setHeadingCorrection);
  }   

    private void configureBindings() {


      //Swerve Commands
      new JoystickButton(p1Controller, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(
        swerve::resetGyro
        ));

    new JoystickButton(p1Controller,XboxController.Button.kBack.value).onFalse(new SwerveCommand(
        swerve,
        () ->
          -MathUtil.applyDeadband(p1Controller.getLeftY(), Controle.DEADBAND),
        () ->
          -MathUtil.applyDeadband(p1Controller.getLeftX(), Controle.DEADBAND),
        () ->
          -MathUtil.applyDeadband(p1Controller.getRightX(), Controle.DEADBAND),
        () -> p1Controller.getRightBumperPressed()
      )
    );

      //Elevator Commands
      new JoystickButton(p2Controller, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.setTarget(1)));

      new JoystickButton(p2Controller, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.setTarget(17)));
    }

  public Command getAutonomousCommand() {
    return null;
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
