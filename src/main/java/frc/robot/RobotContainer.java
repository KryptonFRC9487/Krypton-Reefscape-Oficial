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
import frc.robot.Constants.Trajetoria;
import frc.robot.commands.MoveToPosition;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private SwerveSubsystem swerve = new SwerveSubsystem(
    new File(Filesystem.getDeployDirectory(), "swerve")
  );

  private final XboxController xboxControle = new XboxController(
    Controle.xboxControle
  );

  final CommandXboxController controleTeste = new CommandXboxController(3);


  public RobotContainer() {
    NamedCommands.registerCommand("Intake", new PrintCommand("Intake"));

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
          -MathUtil.applyDeadband(xboxControle.getLeftY(), Controle.DEADBAND),
        () ->
          -MathUtil.applyDeadband(xboxControle.getLeftX(), Controle.DEADBAND),
        () ->
          -MathUtil.applyDeadband(xboxControle.getRightX(), Controle.DEADBAND),
        () -> xboxControle.getRightBumperPressed()
      )
    );
    } else {
      swerve.setDefaultCommand(
      new SwerveCommand(
        swerve,
        () ->
          MathUtil.applyDeadband(xboxControle.getLeftY(), Controle.DEADBAND),
        () ->
          MathUtil.applyDeadband(xboxControle.getLeftX(), Controle.DEADBAND),
        () ->
          MathUtil.applyDeadband(xboxControle.getRightX(), Controle.DEADBAND),
        () -> xboxControle.getRightBumperPressed()
      )
    );
    }

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      new JoystickButton(xboxControle, XboxController.Button.kX.value)
          .onTrue(new MoveToPosition(swerve, 1.630, 7.328));
      new JoystickButton(xboxControle, XboxController.Button.kB.value)
          .onTrue(new MoveToPosition(swerve, 0.707, 1.346));
    } else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      new JoystickButton(xboxControle, XboxController.Button.kB.value)
          .onTrue(new MoveToPosition(swerve, 15.872, 7.364));
      new JoystickButton(xboxControle, XboxController.Button.kX.value)
          .onTrue(new MoveToPosition(swerve, 15.980, 0.758));
    }
  }

  private void configureBindings() {
    if (Robot.isSimulation())
    {
      controleTeste.start().onTrue(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }

    new JoystickButton(xboxControle, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(
        swerve::resetGyro
    ));

    new JoystickButton(xboxControle, XboxController.Button.kY.value)
      .toggleOnTrue(Commands.startEnd(
        swerve::disableHeading, swerve::resetHeading
    ));

    new JoystickButton(xboxControle, XboxController.Button.kRightBumper.value)
    .onTrue(new MoveToPosition(swerve, 8, 4));

    new JoystickButton(xboxControle, 0)
      .onTrue(new MoveToPosition(swerve, 0, 0));
  }

  private void registerAutoCommands() {

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
