package frc.robot.subsystems.auto;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public enum onTheFlyOffsets {
    // Trench Offsets
    TRENCH_FAR(0, 0, 0),
    TRENCH_CLOSE(0, 0, 0),
    // Ramp Offsets
    RAMP_CLOSE(0, 0, 0),
    RAMP_FAR(0, 0, 0),
    // Neutral Zone Offsets
    NEUTRAL_CLOSE(0, 0, 0),
    NEUTRAL_CENTER(0, 0, 0),
    NEUTRAL_FAR(0, 0, 0),
    // Climb Offsets
    CLIMB_LEFT(0, 0, 0),
    CLIMB_RIGHT(0, 0, 0),
    // Human Player Offset
    HUMAN_PLAYER(0, 0, 0),
    // Depot Offset
    DEPOT(0, 0, 0),
    // Default Score Location Offsets
    SCORE_LEFT(0, 0, 0),
    SCORE_CENTER(0, 0, 0),
    SCORE_RIGHT(0, 0, 0);

    public final Transform2d transform;

    onTheFlyOffsets(double x, double y, double angle){
        this.transform = new Transform2d(x, y, new Rotation2d(Degree.of(angle)));
    }
}
