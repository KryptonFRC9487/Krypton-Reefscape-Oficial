// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.GamepadConstants;
import frc.robot.Constants.POV;
import frc.robot.Constants.ReefsConstants.ReefsScorePose;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.teleOp.SwerveCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OuttakePivotSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.ScoreSystem;

public class RobotContainer { 

  private final File swerveConfigFile = new File(Filesystem.getDeployDirectory(), "swerve");

  private final SendableChooser<Command> autoChooser;

  private final VisionSubsystem vision = new VisionSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(swerveConfigFile, vision);
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  private final OuttakeSubsystem m_outtakeSubsystem = new OuttakeSubsystem();
  private final OuttakePivotSubsystem m_outtakePivotSubsystem = new OuttakePivotSubsystem();
  // private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();


  private final ScoreSystem m_scoreSystem = new ScoreSystem(
  m_elevatorSubsystem,
  m_outtakePivotSubsystem,
  m_outtakeSubsystem);

  private XboxController p1Controller = new XboxController(
      GamepadConstants.P1_PORT
  );

  private XboxController p2Controller = new XboxController(
  GamepadConstants.P2_PORT
  );


  public RobotContainer() {
    setDefaultCommands();
    registerAutoCommands();
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser("");
    SmartDashboard.putData("Auto Select", autoChooser);
  }

  private void setDefaultCommands() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      swerveSubsystem.setDefaultCommand(new SwerveCommand(
        swerveSubsystem, 
        () -> -p1Controller.getLeftY(),  
        () -> -p1Controller.getLeftX(), 
        () -> -p1Controller.getRightX(),
        () -> p1Controller.getRightBumperButtonPressed()));
    } else if (DriverStation.getAlliance().get() ==  DriverStation.Alliance.Blue){
      swerveSubsystem.setDefaultCommand(new SwerveCommand(
        swerveSubsystem, 
        () -> -p1Controller.getLeftY(),  
        () -> -p1Controller.getLeftX(), 
        () -> -p1Controller.getRightX(),
        () -> p1Controller.getRightBumperButtonPressed()));
    }

    m_outtakeSubsystem.setDefaultCommand(new OuttakeCommand(m_outtakeSubsystem, p2Controller));
  }

  private void registerAutoCommands() {

    NamedCommands.registerCommand("Expelir Coral", m_outtakeSubsystem.setOuttakeSpeedCmd(0.3));

    NamedCommands.registerCommand("Parar Expelir", m_outtakeSubsystem.setOuttakeSpeedCmd(0));

    NamedCommands.registerCommand("Score Coral - L4", m_scoreSystem.scoreCoralAuto(ReefsScorePose.L4));
    NamedCommands.registerCommand("Elevator Init", m_scoreSystem.scoreCoralAuto(ReefsScorePose.INITAL));

  }

  private void configureBindings() {
    // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
    //   new JoystickButton(p1Controller, XboxController.Button.kX.value)
    //     .whileTrue(
    //       swerveSubsystem.driveToPose(
    //         new Pose2d(
    //           new Translation2d(1.630, 7.328), 
    //           Rotation2d.fromDegrees(0)
    //   )));

    //   new JoystickButton(p1Controller, XboxController.Button.kB.value)
    //     .whileTrue(
    //       swerveSubsystem.driveToPose(
    //         new Pose2d(
    //           new Translation2d(0.707, 1.346), 
    //           Rotation2d.fromDegrees(0)
    //   )));
    // } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
    //   new JoystickButton(p1Controller, XboxController.Button.kX.value)
    //     .whileTrue(
    //       swerveSubsystem.driveToPose(
    //         new Pose2d(
    //           new Translation2d(15.980, 0.758), 
    //           Rotation2d.fromDegrees(0)
    //   )));

    //   new JoystickButton(p1Controller, XboxController.Button.kB.value)
    //     .whileTrue(
    //       swerveSubsystem.driveToPose(
    //         new Pose2d(
    //           new Translation2d( 15.872, 7.364), 
    //           Rotation2d.fromDegrees(0)
    //   )));
    // }
      //   climberSubsystem.setDefaultCommand(
      // new ClimberCommand(
      //   climberSubsystem, p1Controller));
      

    new JoystickButton(p1Controller, XboxController.Button.kBack.value).onFalse(new SwerveCommand(
      swerveSubsystem,
      () -> -MathUtil.applyDeadband(p1Controller.getLeftY(), GamepadConstants.DEADBAND),
      () -> -MathUtil.applyDeadband(p1Controller.getLeftX(), GamepadConstants.DEADBAND),
      () -> -MathUtil.applyDeadband(p1Controller.getRightX(), GamepadConstants.DEADBAND),
      () -> p1Controller.getRightBumperPressed()));

    new JoystickButton(p1Controller, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(swerveSubsystem::resetGyro));

    new POVButton(p2Controller, POV.DOWN).onTrue(m_scoreSystem.scoreCoral(ReefsScorePose.INITAL));
    new POVButton(p2Controller, POV.RIGHT).onTrue(m_scoreSystem.scoreCoral(ReefsScorePose.L2));
    new POVButton(p2Controller, POV.LEFT).onTrue(m_scoreSystem.scoreCoral(ReefsScorePose.L3));
    new POVButton(p2Controller, POV.UP).onTrue(m_scoreSystem.scoreCoral(ReefsScorePose.L4));
    new JoystickButton(p2Controller, Buttons.BUTTON_BACK).onTrue(m_scoreSystem.scoreCoral(ReefsScorePose.CLIMBPOSE));

    // ToPose Commands
    if (Robot.isSimulation()) {
      new JoystickButton(p1Controller, XboxController.Button.kStart.value)
        .onTrue(Commands.runOnce(() -> swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }

    new JoystickButton(p1Controller, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(
            swerveSubsystem::resetGyro));

    new JoystickButton(p1Controller, XboxController.Button.kY.value)
        .toggleOnTrue(Commands.startEnd(
            swerveSubsystem::disableHeading, swerveSubsystem::resetHeading));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  //Heading Correction 
  public void setHeadingCorrection(boolean setHeadingCorrection){
    swerveSubsystem.getSwerveDrive().setHeadingCorrection(setHeadingCorrection);
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return swerveSubsystem;
  }
}