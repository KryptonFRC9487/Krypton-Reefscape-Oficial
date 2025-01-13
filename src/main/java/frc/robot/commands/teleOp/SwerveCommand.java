// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOp;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Dimensoes;
import frc.robot.Constants.StateStrings;
import frc.robot.Constants.SwerveMotionConfig;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * Classe que calcula a partir da entrada do gamepad a saída do swerve
 */
public class SwerveCommand extends Command {

  // Variáveis que guardam nossas funções do gamepad
  DoubleSupplier y;
  DoubleSupplier x;
  DoubleSupplier turn;
  BooleanSupplier toggleSpeed;
  String fastSpeedMode = StateStrings.ON;

  // Objetos necessárias para acessar funções e variáveis
  SwerveSubsystem swerve;
  SwerveController controller;

  // Variáveis que guardam a translação e velocidade angular do swerve
  Translation2d translation;
  double angle;
  double omega;

  public SwerveCommand(
      SwerveSubsystem swerve,
      DoubleSupplier y,
      DoubleSupplier x,
      DoubleSupplier turn,
      BooleanSupplier toggleSpeed) {
    this.y = y;
    this.x = x;
    this.turn = turn;
    this.swerve = swerve;
    this.toggleSpeed = toggleSpeed;

    controller = swerve.getSwerveController();

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (toggleSpeed.getAsBoolean()) {
      fastSpeedMode = fastSpeedMode == StateStrings.OFF ? StateStrings.ON : StateStrings.OFF;
    }

    double multDeVelocidade = fastSpeedMode == StateStrings.ON ? 0.8 : 0.3;

    Drive(multDeVelocidade);
  }

  private void Drive(double multDeVelocidade) {
    Drive(multDeVelocidade, multDeVelocidade, multDeVelocidade);
  }

  private void Drive(
      double multTranslacionalY,
      double multTranslacionalX,
      double multRotacional) {

    double xVelocity = y.getAsDouble() * multTranslacionalY;
    double yVelocity = x.getAsDouble() * multTranslacionalX;
    double angVelocity = turn.getAsDouble() * multRotacional;

    translation = new Translation2d(xVelocity * SwerveMotionConfig.MAX_SPEED, yVelocity * SwerveMotionConfig.MAX_SPEED);

    omega = controller.config.maxAngularVelocity * angVelocity;

    if (SwerveMotionConfig.ACCEL_CORRECTION) {
      translation = SwerveMath.limitVelocity(
          translation,
          swerve.getFieldVelocity(),
          swerve.getPose(),
          Dimensoes.LOOP_TIME,
          Dimensoes.ROBOT_MASS,
          List.of(Dimensoes.CHASSIS),
          swerve.getSwerveDriveConfiguration());
    }

    swerve.drive(translation, omega, SwerveMotionConfig.FIELD_RELATIVE);
  }

  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
