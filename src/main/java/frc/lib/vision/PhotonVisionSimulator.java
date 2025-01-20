package frc.lib.vision;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSimulator extends SubsystemBase {

  VisionSystemSim visionSim;

  public PhotonVisionSimulator() {
    visionSim = new VisionSystemSim("main");

    TargetModel targetModel = new TargetModel(0.5, 0.25); // Dimensões padrão
    Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
    visionSim.addVisionTargets(new VisionTargetSim(targetPose, targetModel));
  }

  @Override
  public void periodic() {
    Pose3d simulatedPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    visionSim.update(simulatedPose);

    SmartDashboard.putData("Simulated Vision System", visionSim.getDebugField());
  }

  public VisionSystemSim getVisionSystem() {
    return visionSim;
  }
}
