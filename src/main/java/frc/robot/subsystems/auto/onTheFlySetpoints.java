package frc.robot.subsystems.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public enum onTheFlySetpoints {
    // All directions are assumed relative to driver station (left far is to the left side of the field, furthest from the driver in their aliance)

    // Trench Locations
    TRENCH_LEFT_NEUTRAL(6, 22, onTheFlyOffsets.TRENCH_OFFSET_NEUTRAL),
    TRENCH_LEFT_ALLIANCE(7, 23, onTheFlyOffsets.TRENCH_OFFSET_ALLIANCE),
    TRENCH_RIGHT_NEUTRAL(1, 17, onTheFlyOffsets.TRENCH_OFFSET_NEUTRAL),
    TRENCH_RIGHT_ALLIANCE(12, 28, onTheFlyOffsets.TRENCH_OFFSET_ALLIANCE),
    // Ramp Locations
    RAMP_LEFT_NEUTRAL(6, 17, onTheFlyOffsets.RAMP_OFFSET_POSITIVE_NEUTRAL),
    RAMP_LEFT_ALLIANCE(7, 28, onTheFlyOffsets.RAMP_OFFSET_NEGATIVE_ALLIANCE),
    RAMP_RIGHT_NEUTRAL(1, 22, onTheFlyOffsets.RAMP_OFFSET_NEGATIVE_NEUTRAL),
    RAMP_RIGHT_ALLIANCE(12, 23, onTheFlyOffsets.RAMP_OFFSET_POSITIVE_ALLIANCE),
    // Neutral Zone Locations
    NEUTRAL_EDGE_LEFT(4, 20, onTheFlyOffsets.NEUTRAL_EDGE_LEFT),
    NEUTRAL_EDGE_MID(4, 20, onTheFlyOffsets.NEUTRAL_EDGE_MID),
    NEUTRAL_EDGE_RIGHT(4, 20, onTheFlyOffsets.NEUTRAL_EDGE_RIGHT),
    NEUTRAL_CENTER_LEFT(4, 20, onTheFlyOffsets.NEUTRAL_CENTER_LEFT),
    NEUTRAL_CENTER_MID(4, 20, onTheFlyOffsets.NEUTRAL_CENTER_MID),
    NEUTRAL_CENTER_RIGHT(4, 20, onTheFlyOffsets.NEUTRAL_CENTER_RIGHT),
    // Climb Locations
    CLIMB_LEFT_SETUP(15, 31, onTheFlyOffsets.CLIMB_LEFT_SETUP),
    CLIMB_LEFT_FINISH(15, 31, onTheFlyOffsets.CLIMB_LEFT_FINISH),
    CLIMB_RIGHT_SETUP(15, 31, onTheFlyOffsets.CLIMB_RIGHT_SETUP),
    CLIMB_RIGHT_FINISH(15, 31, onTheFlyOffsets.CLIMB_RIGHT_FINISH),
    // Human Player Locations
    HUMAN_PLAYER(13, 29, onTheFlyOffsets.HUMAN_PLAYER),
    // Default Score Location Offsets
    SCORE_LEFT(10, 26, onTheFlyOffsets.SCORE_LEFT),
    SCORE_CENTER(10, 26, onTheFlyOffsets.SCORE_CENTER),
    SCORE_RIGHT(10, 26, onTheFlyOffsets.SCORE_RIGHT);

    private final AprilTagFieldLayout fieldLayout;

    public final int redAprilTag;
    public final int blueAprilTag;
    public final onTheFlyOffsets offset;
    public final Pose2d redAlignmentPose;
    public final Pose2d blueAlignmentPose;

    onTheFlySetpoints(int redAprilTag, int blueAprilTag, onTheFlyOffsets offset){
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        this.redAprilTag = redAprilTag;
        this.blueAprilTag = blueAprilTag;
        this.offset = offset;
        this.redAlignmentPose = getAlignmentPose(redAprilTag, this.offset);
        this.blueAlignmentPose = getAlignmentPose(blueAprilTag, this.offset);
    }

    private Pose2d getAlignmentPose(int apriltag, onTheFlyOffsets offset){
        return this.fieldLayout.getTagPose(apriltag).get().toPose2d().transformBy(offset.transform);
    }

    private static Command getOnTheFlyCommands(onTheFlySetpoints targetPose) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
         Pose2d targetPose2d;
            if(alliance.isPresent() && alliance.get() == Alliance.Red) targetPose2d = targetPose.redAlignmentPose;
            else targetPose2d = targetPose.blueAlignmentPose;
        return AutoBuilder.pathfindToPose(
            targetPose2d,
            new PathConstraints(4.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
        }
        
    public static Command alignLeftClimb() {
        return getOnTheFlyCommands(CLIMB_LEFT_SETUP);
    }
    public static Command alignClimbRight() {
        return getOnTheFlyCommands(CLIMB_RIGHT_SETUP);
    }
    public static Command alignHumanPlayer() {
        return getOnTheFlyCommands(HUMAN_PLAYER);
    }
    public static Command alignScoreLeft() {
        return getOnTheFlyCommands(SCORE_LEFT);
    }
    public static Command alignScoreRight() {
        return getOnTheFlyCommands(SCORE_RIGHT);
    }
    public static Command alignScoreCenter() {
        return getOnTheFlyCommands(SCORE_CENTER);
    }
}
    
