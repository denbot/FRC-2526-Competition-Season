package frc.robot.subsystems.auto;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class onTheFlyOffsets {
    public enum trenchOffsets{
        LEFT(getNewTransform(0, 0, 0)),
        RIGHT(getNewTransform(0, 0, 0));

        public final Transform2d transform;

        trenchOffsets(Transform2d transform){
            this.transform = transform;
        }
    }

    public enum neutralZoneOffsets{
        CENTER_TOP(getNewTransform(0, 0, 0)),
        CENTER_MID(getNewTransform(0, 0, 0)),
        CENTER_BOTTOM(getNewTransform(0, 0, 0)),
        CLOSE_TOP(getNewTransform(0, 0, 0)),
        CLOSE_MID(getNewTransform(0, 0, 0)),
        CLOSE_BOTTOM(getNewTransform(0, 0, 0)),
        FAR_TOP(getNewTransform(0, 0, 0)),        
        FAR_MID(getNewTransform(0, 0, 0)),        
        FAR_BOTTOM(getNewTransform(0, 0, 0));
        
        public final Transform2d transform;

        neutralZoneOffsets(Transform2d transform){
            this.transform = transform;
        }
    }

    private static Transform2d getNewTransform(double x, double y, double angle){
        return new Transform2d(x, y, new Rotation2d(Degree.of(angle)));
    }
}
