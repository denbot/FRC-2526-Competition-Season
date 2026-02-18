package frc.robot.subsystems.vision;

import frc.robot.subsystems.drive.Drive;

public class LimelightIOSim implements LimelightIO{
    //TODO add limelight sim functionality
    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        inputs.backLeftConnected = false;
        inputs.backRightConnected = false;
        inputs.frontConnected = false;
        inputs.allConnected = false;

        inputs.backLeftTagCount = 0;
        inputs.backRightTagCount = 0;
        inputs.frontTagCount = 0;
        inputs.totalTagCount = 0;
    }

    public void getAllPoseEstimate(Drive drive) {}

}
