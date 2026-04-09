package frc.robot.subsystems.auto;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public enum onTheFlyOffsets {
    TRENCH_OFFSET_NEUTRAL(1, 0, 0),
    TRENCH_OFFSET_ALLIANCE(1.75, 0, 180),

    RAMP_OFFSET_NEGATIVE_NEUTRAL(1.75, -1.8, -90),
    RAMP_OFFSET_NEGATIVE_ALLIANCE(1.75, -1.8, 90),
    RAMP_OFFSET_POSITIVE_NEUTRAL(1.75, 1.8, -90),
    RAMP_OFFSET_POSITIVE_ALLIANCE(1.75, 1.8, 90),
    // Neutral Zone Offsets
    NEUTRAL_EDGE_MID_FROM_LEFT(2.5, 0, -90),
    NEUTRAL_EDGE_MID_FROM_RIGHT(2.5, 0, 90),
    NEUTRAL_EDGE_LEFT(2.5, 2.75, -90),
    NEUTRAL_EDGE_RIGHT(2.5, -2.75, 90),
    NEUTRAL_CENTER_MID_FROM_LEFT(3, 0, -90),
    NEUTRAL_CENTER_MID_FROM_RIGHT(3, 0, 90),
    NEUTRAL_CENTER_LEFT(3, 2.75, -90),
    NEUTRAL_CENTER_RIGHT(3, -2.75, 90),
    // Climb Offsets
    CLIMB_LEFT_SETUP(2.5, 0.4, 0),
    CLIMB_LEFT_FINISH(1.65, 0.4, 0),
    CLIMB_RIGHT_SETUP(2.5, -0.4, 0),
    CLIMB_RIGHT_FINISH(1.65, -0.4, 0),
    HUMAN_PLAYER(0.648, 0, 180),
    // Default Score Location Offsets
    SCORE_LEFT(1, -1.3, -30),
    SCORE_RIGHT(1, 1.3, 30),
    SCORE_CENTER(1, 0, 180);

    public final Transform2d transform;

    onTheFlyOffsets(double x, double y, double angle){
        this.transform = new Transform2d(x, y, new Rotation2d(Degree.of(angle)));
    }
}
