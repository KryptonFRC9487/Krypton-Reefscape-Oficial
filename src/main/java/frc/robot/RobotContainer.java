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
import frc.robot.Constants.GamepadConstants;
import frc.robot.Constants.Trajetoria;
import frc.robot.commands.MoveToPosition;
import frc.robot.commands.OuttakeCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.teleOp.SwerveCommand;

public class RobotContainer {

  private final File swerveConfigFile = new File(Filesystem.getDeployDirectory(), "swerve");

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(swerveConfigFile);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final OuttakeSubsystem outtakeSubsystem = new OuttakeSubsystem();

  private XboxController p1Controller = new XboxController(
      GamepadConstants.P1_PORT
  );

  private XboxController p2Controller = new XboxController(
      GamepadConstants.P2_PORT
  );

  private final CommandXboxController controleTeste = new CommandXboxController(3);

  public RobotContainer() {
    setDefaultCommands();
    registerAutoCommands();
    configureBindings();
  }

  private void setDefaultCommands() {
    swerveSubsystem.setDefaultCommand(new SwerveCommand(
        swerveSubsystem,
        () -> MathUtil.applyDeadband(p1Controller.getLeftY(), GamepadConstants.DEADBAND),
        () -> MathUtil.applyDeadband(p1Controller.getLeftX(), GamepadConstants.DEADBAND),
        () -> MathUtil.applyDeadband(p1Controller.getRightX(), GamepadConstants.DEADBAND),
        () -> p1Controller.getRightBumperPressed()));

    outtakeSubsystem.setDefaultCommand(
      new OuttakeCommand(
        outtakeSubsystem, p2Controller
      )
    );
  }

  private void registerAutoCommands() {
    NamedCommands.registerCommand("Intake", new PrintCommand("Intake"));
  }

  private void configureBindings() {

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      new JoystickButton(p1Controller, XboxController.Button.kX.value)
          .onTrue(new MoveToPosition(swerveSubsystem, 1.630, 7.328));
      new JoystickButton(p1Controller, XboxController.Button.kB.value)
          .onTrue(new MoveToPosition(swerveSubsystem, 0.707, 1.346));
    } else {
      new JoystickButton(p1Controller, XboxController.Button.kX.value)
          .onTrue(new MoveToPosition(swerveSubsystem, 15.980, 0.758));
      new JoystickButton(p1Controller, XboxController.Button.kB.value)
          .onTrue(new MoveToPosition(swerveSubsystem, 15.872, 7.364));
    }

    new JoystickButton(p1Controller, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(swerveSubsystem::resetGyro));

    // Elevator Commands
    new JoystickButton(p2Controller, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.setTarget(1)));

    new JoystickButton(p2Controller, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.setTarget(17)));


    // ToPose Commands
    if (Robot.isSimulation()) {
      controleTeste.start()
          .onTrue(Commands.runOnce(() -> swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }

    new JoystickButton(p1Controller, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(
            swerveSubsystem::resetGyro));

    new JoystickButton(p1Controller, XboxController.Button.kY.value)
        .toggleOnTrue(Commands.startEnd(
            swerveSubsystem::disableHeading, swerveSubsystem::resetHeading));

    new JoystickButton(p1Controller, XboxController.Button.kRightBumper.value)
        .onTrue(new MoveToPosition(swerveSubsystem, 8, 4));
  }

  public Command getAutonomousCommand() {
    return swerveSubsystem.getAutonomousCommand(Trajetoria.NOME_TRAJETORIA1, true);
  }

  //Heading Correction 
  public void setHeadingCorrection(boolean setHeadingCorrection){
    swerveSubsystem.swerveDrive.setHeadingCorrection(setHeadingCorrection);
  }
}
