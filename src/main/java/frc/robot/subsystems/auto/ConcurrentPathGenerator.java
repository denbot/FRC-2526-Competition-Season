package frc.robot.subsystems.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConcurrentPathGenerator {
    public Command getConcurrentPath(onTheFlySetpoints... setpoints){
        SequentialCommandGroup finalPath = new SequentialCommandGroup();
        Optional<Alliance> alliance = DriverStation.getAlliance();

        for(onTheFlySetpoints setpoint: setpoints){
            Pose2d targetPose;
            if(alliance.isPresent() && alliance.get() == Alliance.Red) targetPose = setpoint.redAlignmentPose;
            else targetPose = setpoint.blueAlignmentPose;

            finalPath.addCommands(
                AutoBuilder.pathfindToPose(targetPose,
                new PathConstraints(6.0, 6.0, Units.degreesToRadians(540), Units.degreesToRadians(720)))
            );
        }
        return finalPath;
    }
}
