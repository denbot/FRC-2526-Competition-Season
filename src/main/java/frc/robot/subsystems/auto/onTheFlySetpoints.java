package frc.robot.subsystems.auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.auto.onTheFlyOffsets.trenchOffsets;

public class onTheFlySetpoints {
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // Trench Locations
    public enum trenchSetpoints{
        TOP_LEFT(7, 17, trenchOffsets.LEFT),
        TOP_RIGHT(6, 28, trenchOffsets.RIGHT),
        BOT_LEFT(12, 22, trenchOffsets.LEFT),
        BOT_RIGHT(1, 23, trenchOffsets.RIGHT);

        public final int redAprilTag;
        public final int blueAprilTag;
        public final trenchOffsets offset;
        public final Pose2d redAlignment;
        public final Pose2d blueAlignment;

        trenchSetpoints(int redAprilTag, int blueAprilTag, trenchOffsets offset){
            this.redAprilTag = redAprilTag;
            this.blueAprilTag = blueAprilTag;
            this.offset = offset;
            this.redAlignment = getAlignmentPose(redAprilTag, this.offset.transform);
            this.blueAlignment = getAlignmentPose(blueAprilTag, this.offset.transform);
        }
    }
    
    // 
    
    private static Pose2d getAlignmentPose(int apriltag, Transform2d offset){
        return fieldLayout.getTagPose(apriltag).get().toPose2d().transformBy(offset);
    }
}
