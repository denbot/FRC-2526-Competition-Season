package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.drive.Drive;

public interface LimelightIO {
    @AutoLog
    public static class LimelightIOInputs{
        public boolean backLeftConnected = false;
        public boolean backRightConnected = false;
        public boolean frontConnected = false;
        public boolean allConnected = false;

        public int backLeftTagCount = 0;
        public int backRightTagCount = 0;
        public int frontTagCount = 0;
        public int totalTagCount = 0;
    }

    public default void updateInputs(LimelightIOInputs inputs) {}

    public default void getAllPoseEstimate(Drive drive) {}
}
