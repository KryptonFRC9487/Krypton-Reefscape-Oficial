package frc.robot.utils;

import frc.robot.Constants.ElevatorConstants.ElevatorPose;
import frc.robot.Constants.OuttakeConstants.OuttakePose;

public class SubsystemTracker {
    private ElevatorPose elevatorPose;
    private double elevatorRealPosition;

    private OuttakePose outtakePose;
    private double outtakeRealPosition;
    
    public void updateElevatorRealPosition(ElevatorPose elevatorPose, double elevatorRealPosition) {
        this.elevatorPose = elevatorPose;
        this.elevatorRealPosition = elevatorRealPosition;
    }

    public ElevatorPose getElevatorPose() {
        return this.elevatorPose;
    }

    public double getElevatorRealPosition() {
        return this.elevatorRealPosition;
    }

    public void updateOuttakeRealPosition(OuttakePose outtakePose, double outtakeRealPosition) {
        this.outtakePose = outtakePose;
        this.outtakeRealPosition = outtakeRealPosition;
    }

    public OuttakePose getOuttakePose() {
        return this.outtakePose;
    }

    public double getOuttakeRealPosition() {
        return this.outtakeRealPosition;
    }
}