// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  CameraServer cameraServer;

  public Robot() {
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    // m_sysIdRoutineBot = new SysIdRoutineBot();

    if (isSimulation() || isTeleop()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("RSL", RobotController.getRSLState());
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.getSwerveSubsystem().resetGyro();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.setHeadingCorrection(true);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();

    }
  }

  @Override
  public void autonomousPeriodic() {

    // m_robotContainer.getSwerveSubsystem().updateOdometry();

  }

  @Override
  public void teleopInit() {

    // m_robotContainer.getSwerveSubsystem().resetGyro();
    m_robotContainer.setHeadingCorrection(true);

  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}