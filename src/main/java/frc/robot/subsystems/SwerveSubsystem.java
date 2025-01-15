// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Tracao;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

/**
 * Classe de subsistema onde fazemos a ponte do nosso código para YAGSL
 */
public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;
  public boolean correctionPID = false;

  // Objeto global autônomo
  // ConfigAuto autonomo;

  // Método construtor da classe
  public SwerveSubsystem(File directory) {

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Tracao.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    // autonomo = new ConfigAuto(this);

    // autonomo.setupPathPlanner();

    swerveDrive.setHeadingCorrection(true);
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
  }

  public void driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    swerveDrive.driveFieldOriented(velocity.get());
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    return swerveDrive.swerveController.getTargetSpeeds(
        xInput,
        yInput,
        headingX,
        headingY,
        getHeading().getRadians());
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput) {
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, 0, 0, 0, Tracao.MAX_SPEED);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  public SwerveController getSwerveController() {
    return swerveDrive.getSwerveController();
  }

  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  public void resetOdometry(Pose2d posicao) {
    swerveDrive.resetOdometry(posicao);
  }

  public void resetGyro() {
    swerveDrive.zeroGyro();
  }

  public void resetHeading() {
    swerveDrive.setHeadingCorrection(true);
    correctionPID = true;
  }

  public void disableHeading() {
    correctionPID = false;
    swerveDrive.setHeadingCorrection(false);
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }


  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    var desiredDeltaPose = new Pose2d(
        speeds.vxMetersPerSecond * Tracao.DT,
        speeds.vyMetersPerSecond * Tracao.DT,
        new Rotation2d(speeds.omegaRadiansPerSecond * Tracao.DT * Tracao.constantRotation));

    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / Tracao.DT), (twist.dy / Tracao.DT), (speeds.omegaRadiansPerSecond));
  }

  public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {

    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  // public void drive(Translation2d translation2d, int i, boolean b) {
  //   // TODO Auto-generated method stub
  //   throw new UnsupportedOperationException("Unimplemented method 'drive'");
  // }


}
