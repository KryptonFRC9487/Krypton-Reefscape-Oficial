package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{

    private final NetworkTable limelightTable;

    public VisionSubsystem(){
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public Pose2d getEstimatedPose(){

        double[] botpose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);

        if(botpose.length < 6){
            return null; //Retorna nulo se a leitura não for válida
        }

        double x = botpose[0];  //Posição X em metros 
        double y = botpose[1];  //Posição Y em metros 
        double angle = botpose[5];  //Ângulo de rotação (Yaw)

        return new Pose2d(new Translation2d(x,y), Rotation2d.fromDegrees(angle));
    }
}
