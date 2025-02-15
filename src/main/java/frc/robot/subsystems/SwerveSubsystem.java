// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.core.CorePigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.LimelightHelpers;
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
  private final CorePigeon2 pigeon;


  // Método construtor da classe
  public SwerveSubsystem(File directory) {

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Tracao.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // autonomo.setupPathPlanner();
    pigeon = new CorePigeon2(13);

    swerveDrive.setHeadingCorrection(true);

    setupPathPlanner();
  }

    /**
   * Setup AutoBuilder for PathPlanner.
   */
  
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
public void periodic() {
    swerveDrive.updateOdometry();
    
    // updateOdometry();

    LimelightHelpers.SetRobotOrientation("limelight" ,
    robotYawInDegrees(),  
    getYawRate(),
    0, 0, 0, 0);
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



  public Command driveToPose(Pose2d pose) {
  // Create the constraints to use while pathfinding
      PathConstraints constraints = new PathConstraints(
          swerveDrive.getMaximumChassisVelocity(), 4.0,
          swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

  // Since AutoBuilder is configured, we can use it to build pathfinding commands
      return AutoBuilder.pathfindToPose(
          pose,
          constraints,
          edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                      );
  }

  public double robotYawInDegrees(){
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      return 0.0;
    } else {
      return 1800.0;
    }
  }

  public double getYawRate(){
    return pigeon.getAngularVelocityZWorld().getValueAsDouble();
  }

//  public void updateOdometry(){
//       poseEstimator.update(
//         Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble()),
//         swerveDrive.getModulePositions());


//       boolean useMegatag2 = true;
//       boolean doRejectUpdate = false;
     
//       if (useMegatag2 == false) {
//         LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");


//         if (mt1.tagCount == 1) {
//           if (mt1.rawFiducials[0].ambiguity > .7) {
//             doRejectUpdate = true;
//           }
//           if (mt1.rawFiducials[0].ambiguity > 3) {
//             doRejectUpdate = true;
//           }
//         }
       
//         if (mt1.tagCount == 0) {
//           doRejectUpdate = true;
//         }


//         if (!doRejectUpdate) {
//           System.out.println("Adicionando medição de visão");
//           poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
//           poseEstimator.addVisionMeasurement(
//             mt1.pose,
//             mt1.timestampSeconds);
//         }
//       } else if (useMegatag2 == true) {
//         LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
//         LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");


//         if (swerveDrive.getFieldVelocity().omegaRadiansPerSecond > 50) {
//           doRejectUpdate = true;
//         }
//         if (mt2.tagCount == 0) {
//           doRejectUpdate = true;
//         }
//         if (!doRejectUpdate) {
//             poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
//             poseEstimator.addVisionMeasurement(
//               mt2.pose,
//               mt2.timestampSeconds);
//         }
//       }
//     }
}
