package frc.robot.subsystems.auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public enum onTheFlySetpoints {
    // All directions are assumed relative to driver station (left far is to the left side of the field, furthest from the driver in their aliance)

    // Trench Locations
    TRENCH_LEFT_NEUTRAL(6, 17, onTheFlyOffsets.TRENCH_OFFSET_NEUTRAL),
    TRENCH_LEFT_ALLIANCE(7, 28, onTheFlyOffsets.TRENCH_OFFSET_ALLIANCE),
    TRENCH_RIGHT_NEUTRAL(1, 22, onTheFlyOffsets.TRENCH_OFFSET_NEUTRAL),
    TRENCH_RIGHT_ALLIANCE(12, 23, onTheFlyOffsets.TRENCH_OFFSET_ALLIANCE),
    // Ramp Locations
    RAMP_LEFT_NEUTRAL(6, 17, onTheFlyOffsets.RAMP_OFFSET_POSITIVE_NEUTRAL),
    RAMP_LEFT_ALIANCE(7, 28, onTheFlyOffsets.RAMP_OFFSET_NEGATIVE_ALLIANCE),
    RAMP_RIGHT_NEUTRAL(1, 22, onTheFlyOffsets.RAMP_OFFSET_NEGATIVE_NEUTRAL),
    RAMP_RIGHT_ALIANCE(12, 23, onTheFlyOffsets.RAMP_OFFSET_POSITIVE_ALLIANCE),
    // Neutral Zone Locations
    // Closest April Tag Alignment handles Y offset, only X offset is required
    NEUTRAL_EDGE_LEFT(4, 20, onTheFlyOffsets.NEUTRAL_EDGE_LEFT),
    NEUTRAL_EDGE_MID(4, 20, onTheFlyOffsets.NEUTRAL_EDGE_MID),
    NEUTRAL_EDGE_RIGHT(4, 20, onTheFlyOffsets.NEUTRAL_EDGE_RIGHT),
    NEUTRAL_CENTER_LEFT(4, 20, onTheFlyOffsets.NEUTRAL_CENTER_LEFT),
    NEUTRAL_CENTER_MID(4, 20, onTheFlyOffsets.NEUTRAL_CENTER_MID),
    NEUTRAL_CENTER_RIGHT(4, 20, onTheFlyOffsets.NEUTRAL_CENTER_RIGHT),
    // Climb Locations
    CLIMB_LEFT(15, 31, onTheFlyOffsets.CLIMB_LEFT),
    CLIMB_RIGHT(15, 31, onTheFlyOffsets.CLIMB_RIGHT),
    // Human Player Locations
    HUMAN_PLAYER(13, 29, onTheFlyOffsets.HUMAN_PLAYER),
    // Depot Player Locations
    DEPOT(7, 23, onTheFlyOffsets.DEPOT),
    // Default Score Location Offsets
    SCORE_LEFT(10, 26, onTheFlyOffsets.SCORE_LEFT),
    SCORE_CENTER(10, 26, onTheFlyOffsets.SCORE_CENTER),
    SCORE_RIGHT(10, 26, onTheFlyOffsets.SCORE_RIGHT);

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
