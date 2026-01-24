package frc.robot.subsystems.auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public enum onTheFlySetpoints {
    // All directions are assumed relative to driver station (left far is to the left side of the field, furthest from the driver in their aliance)

    // Trench Locations
    TRENCH_LEFT_FAR(6, 28, onTheFlyOffsets.TRENCH_OFFSET),
    TRENCH_LEFT_CLOSE(7, 17, onTheFlyOffsets.TRENCH_OFFSET),
    TRENCH_RIGHT_FAR(1, 23, onTheFlyOffsets.TRENCH_OFFSET),
    TRENCH_RIGHT_CLOSE(12, 22, onTheFlyOffsets.TRENCH_OFFSET),
    // Ramp Locations
    RAMP_LEFT_FAR(5, 22, onTheFlyOffsets.RAMP_OFFSET),
    RAMP_LEFT_CLOSE(8, 23, onTheFlyOffsets.RAMP_OFFSET),
    RAMP_RIGHT_FAR(2, 17, onTheFlyOffsets.RAMP_OFFSET),
    RAMP_RIGHT_CLOSE(12, 28, onTheFlyOffsets.RAMP_OFFSET),
    // Neutral Zone Locations
    // Closest April Tag Alignment handles Y offset, only X offset is required
    NEUTRAL_CLOSE_TOP(6, 17, onTheFlyOffsets.NEUTRAL_OFF_CENTER),
    NEUTRAL_CLOSE_MID(4, 20, onTheFlyOffsets.NEUTRAL_OFF_CENTER),
    NEUTRAL_CLOSE_BOTTOM(1, 22, onTheFlyOffsets.NEUTRAL_OFF_CENTER),
    NEUTRAL_CENTER_TOP(6, 17, onTheFlyOffsets.NEUTRAL_CENTER),
    NEUTRAL_CENTER_MID(4, 20, onTheFlyOffsets.NEUTRAL_CENTER),
    NEUTRAL_CENTER_BOTTOM(1, 22, onTheFlyOffsets.NEUTRAL_CENTER),
    NEUTRAL_FAR_TOP(17, 6, onTheFlyOffsets.NEUTRAL_OFF_CENTER),
    NEUTRAL_FAR_MID(20, 4, onTheFlyOffsets.NEUTRAL_OFF_CENTER),
    NEUTRAL_FAR_BOTTOM(22, 1, onTheFlyOffsets.NEUTRAL_OFF_CENTER),
    // Climb Locations
    CLIMB_LEFT(16, 32, onTheFlyOffsets.CLIMB_LEFT),
    CLIMB_RIGHT(15, 31, onTheFlyOffsets.CLIMB_RIGHT),
    // Human Player Locations
    HUMAN_PLAYER(13, 29, onTheFlyOffsets.HUMAN_PLAYER),
    // Depot Player Locations
    DEPOT(7, 23, onTheFlyOffsets.DEPOT),
    // Default Score Location Offsets
    SCORE_LEFT(8, 24, onTheFlyOffsets.SCORE_OFF_CENTER),
    SCORE_CENTER(9, 25, onTheFlyOffsets.SCORE_CENTER),
    SCORE_RIGHT(11, 27, onTheFlyOffsets.SCORE_OFF_CENTER);

    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public final int redAprilTag;
    public final int blueAprilTag;
    public final onTheFlyOffsets offset;
    public final Pose2d redAlignmentPose;
    public final Pose2d blueAlignmentPose;

    onTheFlySetpoints(int redAprilTag, int blueAprilTag, onTheFlyOffsets offset){
        this.redAprilTag = redAprilTag;
        this.blueAprilTag = blueAprilTag;
        this.offset = offset;
        this.redAlignmentPose = getAlignmentPose(redAprilTag, this.offset);
        this.blueAlignmentPose = getAlignmentPose(blueAprilTag, this.offset);
    }

    private static Pose2d getAlignmentPose(int apriltag, onTheFlyOffsets offset){
        return fieldLayout.getTagPose(apriltag).get().toPose2d().transformBy(offset.transform);
    }
}
