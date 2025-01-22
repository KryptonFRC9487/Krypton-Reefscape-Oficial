// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ElevatorConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private final SparkMax leftMotor, rightMotor;
  private final XboxController controller;

  public Robot() {
    leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    controller = new XboxController(0);
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
  }

  @Override
  public void teleopPeriodic() {
    leftMotor.set(controller.getLeftY());
    rightMotor.set(-controller.getRightY());

    SmartDashboard.putNumber("L. Elevator Position (Rotations)", leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("L. Elevator Velocity (Rotations per Second)", leftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("L. Elevator Applied Voltage", leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
    SmartDashboard.putNumber("R. Elevator Applied Output", leftMotor.getAppliedOutput());

    SmartDashboard.putNumber("R. Elevator Position (Rotations)", rightMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("R. Elevator Velocity (Rotations per Second)", rightMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("R. Elevator Applied Voltage", rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());
    SmartDashboard.putNumber("R. Elevator Applied Output", rightMotor.getAppliedOutput());
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
