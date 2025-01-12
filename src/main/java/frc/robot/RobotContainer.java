// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controle;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Trajetoria;
import frc.robot.commands.MoveToPosition;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private SwerveSubsystem swerve = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );


  private XboxController p1Controller = new XboxController(
    Controle.P1PORT
  );

  private XboxController p2Controller = new XboxController(
    Controle.P2PORT
  );

  final CommandXboxController controleTeste = new CommandXboxController(3);

  private ElevatorSubsystem elevatorSubsystem;


  public RobotContainer() {
    NamedCommands.registerCommand("Intake", new PrintCommand("Intake"));

    elevatorSubsystem = new ElevatorSubsystem();

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

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      new JoystickButton(p1Controller, XboxController.Button.kX.value)
          .onTrue(new MoveToPosition(swerve, 1.630, 7.328));
      new JoystickButton(p1Controller, XboxController.Button.kB.value)
          .onTrue(new MoveToPosition(swerve, 0.707, 1.346));
    } else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      new JoystickButton(p1Controller, XboxController.Button.kB.value)
          .onTrue(new MoveToPosition(swerve, 15.872, 7.364));
      new JoystickButton(p1Controller, XboxController.Button.kX.value)
          .onTrue(new MoveToPosition(swerve, 15.980, 0.758));
    }
  }



  private void registerAutoCommands() {

  }

  //Heading Correction 
 

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


      //ToPose Commands 
        if (Robot.isSimulation())
        {
          controleTeste.start().onTrue(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
        }
    
        new JoystickButton(p1Controller, XboxController.Button.kA.value)
          .onTrue(new InstantCommand(
            swerve::resetGyro
        ));
    
        new JoystickButton(p1Controller, XboxController.Button.kY.value)
          .toggleOnTrue(Commands.startEnd(
            swerve::disableHeading, swerve::resetHeading
        ));
    
        new JoystickButton(p1Controller, XboxController.Button.kRightBumper.value)
        .onTrue(new MoveToPosition(swerve, 8, 4));
    
        new JoystickButton(p1Controller, 0)
          .onTrue(new MoveToPosition(swerve, 0, 0));
    }

  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand(Trajetoria.NOME_TRAJETORIA, true);
  }

  //Heading Correction 
  public void setHeadingCorrection(boolean setHeadingCorrection){
    swerve.swerveDrive.setHeadingCorrection(setHeadingCorrection);
  }   

  // Define os motores como coast ou brake


  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}
