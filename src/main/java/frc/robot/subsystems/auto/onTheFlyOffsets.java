package frc.robot.subsystems.auto;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public enum onTheFlyOffsets {
    TRENCH_OFFSET_NEUTRAL(1.75, 0, 90),
    TRENCH_OFFSET_ALLIANCE(1.75, 0, -90),
    RAMP_OFFSET_NEGATIVE_NEUTRAL(1.75, -1.8, 180),
    RAMP_OFFSET_NEGATIVE_ALLIANCE(1.75, -1.8, 0),
    RAMP_OFFSET_POSITIVE_NEUTRAL(1.75, 1.8, 180),
    RAMP_OFFSET_POSITIVE_ALLIANCE(1.75, 1.8, 0),
    // Neutral Zone Offsets
    NEUTRAL_EDGE_MID(2.5, 0, 0),
    NEUTRAL_EDGE_LEFT(2.5, 2.5, 90),
    NEUTRAL_EDGE_RIGHT(2.5, -2.5, -90),
    NEUTRAL_CENTER_MID(3, 0, 0),
    NEUTRAL_CENTER_LEFT(3, 2.5, 90),
    NEUTRAL_CENTER_RIGHT(3, -2.5, -90),
    // Climb Offsets
    CLIMB_LEFT_SETUP(1.5, 0.5, 180),
    CLIMB_LEFT_FINISH(1, 0.5, 180),
    CLIMB_RIGHT_SETUP(1.5, -0.5, 180),
    CLIMB_RIGHT_FINISH(1, -0.5, 180),
    HUMAN_PLAYER(0.5, 0, 0),
    // Default Score Location Offsets
    SCORE_LEFT(2, -2, 45),
    SCORE_RIGHT(2, 2, -45),
    SCORE_CENTER(2, 0, 0);

    public final Transform2d transform;

    onTheFlyOffsets(double x, double y, double angle){
        this.transform = new Transform2d(x, y, new Rotation2d(Degree.of(angle)));
    }
}
