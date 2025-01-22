// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

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
  private final XboxController xboxController;

  public Robot() {
    leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    xboxController = new XboxController(1);

  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    if (isSimulation())
    {
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
    leftMotor.set(xboxController.getLeftY());
    rightMotor.set(-xboxController.getLeftY());

    SmartDashboard.putNumber("Left Elevator Position (Rotations)", leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Left Elevator Velocity (Rotations per Second)", leftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Left Elevator Voltage", leftMotor.getBusVoltage());

    SmartDashboard.putNumber("Right Elevator Position (Rotations)", rightMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Elevator Velocity (Rotations per Second)", rightMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Elevator Voltage", rightMotor.getBusVoltage());
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
