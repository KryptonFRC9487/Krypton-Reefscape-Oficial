// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  Spark m_led = new Spark(0);
  XboxController con = new XboxController(3);

  public Robot() {

  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    CameraServer.startAutomaticCapture();
  }
  

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putBoolean("RSL", RobotController.getRSLState());
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());   

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    m_robotContainer.setHeadingCorrection(true);

    m_robotContainer.getSwerveSubsystem().resetGyro();
  }

  @Override
  public void teleopPeriodic() {
    if (con.getRawButton(XboxController.Button.kA.value)) {
      m_led.set(0.69);
    }
    if (con.getRawButton(XboxController.Button.kB.value)) {
      m_led.set(0.61);
    }
    if (con.getRawButton(XboxController.Button.kY.value)) {
      m_led.set(0.77);
    }
    if (con.getRawButton(XboxController.Button.kX.value)) {
      m_led.set(0.143);
    }

    //A - verde
    //B - azul
    //Y - rosa

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
