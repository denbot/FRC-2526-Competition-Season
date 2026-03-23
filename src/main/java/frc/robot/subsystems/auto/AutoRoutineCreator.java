package frc.robot.subsystems.auto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoRoutineCreator {
    private static driverRelative startingSide = driverRelative.LEFT;
    private static boolean isBlue = true;
    private static Intake intake;
    public static Shooter shooter;
    private static Indexer indexer;

    public enum driverRelative{
        LEFT,
        CENTER,
        RIGHT,
        CLOSE,
        FAR,
        TRENCH,
        RAMP;
    }

    public static void addSubsystems(Intake intake, Shooter shooter, Indexer indexer){
        AutoRoutineCreator.intake = intake;
        AutoRoutineCreator.shooter = shooter;
        AutoRoutineCreator.indexer = indexer;
    }

    public static void setStartingSide(driverRelative side){
        AutoRoutineCreator.startingSide = side;
    }

    public static void setIsBlue(boolean isBlue){
        AutoRoutineCreator.isBlue = isBlue;
    }

    public static void addSweep(driverRelative sweepDistance, boolean doFishtail){
        double approachingAngle = isBlue?90:270;
        double returningAngle = isBlue?270:90;

        // align to setup pose
        AutoCommandHelper.addSetpoint(
            setpointToPose(startingSide==driverRelative.LEFT
                ?sweepDistance==driverRelative.FAR?onTheFlySetpoints.NEUTRAL_CENTER_LEFT:onTheFlySetpoints.NEUTRAL_EDGE_LEFT
                :sweepDistance==driverRelative.FAR?onTheFlySetpoints.NEUTRAL_CENTER_RIGHT:onTheFlySetpoints.NEUTRAL_EDGE_RIGHT, 
                approachingAngle));
        
        // start running intake
        AutoCommandHelper.addSubsystemAction(intake.setIntakeMaxLength());
        AutoCommandHelper.addSubsystemAction(intake.runIntake(RotationsPerSecond.of(80)));
                
        // ensure the sweep is run at a slower speed
        AutoCommandHelper.setMaxLinearVelocity(MetersPerSecond.of(2));

        // sweep straight
        AutoCommandHelper.addSetpoint(
            setpointToPose(startingSide==driverRelative.LEFT
                ?sweepDistance==driverRelative.FAR?onTheFlySetpoints.NEUTRAL_CENTER_MID_FROM_LEFT:onTheFlySetpoints.NEUTRAL_EDGE_MID_FROM_LEFT
                :sweepDistance==driverRelative.FAR?onTheFlySetpoints.NEUTRAL_CENTER_MID_FROM_RIGHT:onTheFlySetpoints.NEUTRAL_EDGE_MID_FROM_RIGHT, 
                approachingAngle));
        
        if(doFishtail){
            // distance ?s are inverted to CLOSE instead of FAR return on the opposite side you start
            AutoCommandHelper.addSetpoint(
                setpointToPose(startingSide==driverRelative.LEFT
                    ?sweepDistance==driverRelative.CLOSE?onTheFlySetpoints.NEUTRAL_CENTER_LEFT:onTheFlySetpoints.NEUTRAL_EDGE_LEFT
                    :sweepDistance==driverRelative.CLOSE?onTheFlySetpoints.NEUTRAL_CENTER_RIGHT:onTheFlySetpoints.NEUTRAL_EDGE_RIGHT, 
                    returningAngle));
        }
        else{
            // Return straight back
            AutoCommandHelper.setMaxLinearVelocity(MetersPerSecond.of(4));
            AutoCommandHelper.addSetpoint(
                setpointToPose(startingSide==driverRelative.LEFT
                    ?sweepDistance==driverRelative.FAR?onTheFlySetpoints.NEUTRAL_CENTER_LEFT:onTheFlySetpoints.NEUTRAL_EDGE_LEFT
                    :sweepDistance==driverRelative.FAR?onTheFlySetpoints.NEUTRAL_CENTER_RIGHT:onTheFlySetpoints.NEUTRAL_EDGE_RIGHT, 
                    approachingAngle));
        }

        // stop running intake
        AutoCommandHelper.setMaxLinearVelocity(MetersPerSecond.of(4));
        AutoCommandHelper.addSubsystemAction(intake.stopIntake());
    }

    public static void addExitAlliance(driverRelative exitLocation){
        // Align to start pose
        AutoCommandHelper.addSetpoint(
            setpointToPose(startingSide==driverRelative.LEFT
                ?exitLocation==driverRelative.TRENCH?onTheFlySetpoints.TRENCH_LEFT_ALLIANCE:onTheFlySetpoints.RAMP_LEFT_ALLIANCE
                :exitLocation==driverRelative.TRENCH?onTheFlySetpoints.TRENCH_RIGHT_ALLIANCE:onTheFlySetpoints.RAMP_RIGHT_ALLIANCE));
        
        // Align to end pose
        AutoCommandHelper.addSetpoint(
            setpointToPose(startingSide==driverRelative.LEFT
                ?exitLocation==driverRelative.TRENCH?onTheFlySetpoints.TRENCH_LEFT_NEUTRAL:onTheFlySetpoints.RAMP_LEFT_NEUTRAL
                :exitLocation==driverRelative.TRENCH?onTheFlySetpoints.TRENCH_RIGHT_NEUTRAL:onTheFlySetpoints.RAMP_RIGHT_NEUTRAL));
    }

    public static void addReturnAlliance(driverRelative returnLocation){
        // Align to start pose
        AutoCommandHelper.addSetpoint(
            setpointToPose(startingSide==driverRelative.LEFT
                ?returnLocation==driverRelative.TRENCH?onTheFlySetpoints.TRENCH_LEFT_NEUTRAL:onTheFlySetpoints.RAMP_LEFT_NEUTRAL
                :returnLocation==driverRelative.TRENCH?onTheFlySetpoints.TRENCH_RIGHT_NEUTRAL:onTheFlySetpoints.RAMP_RIGHT_NEUTRAL));

        // Align to end pose
        AutoCommandHelper.addSetpoint(
            setpointToPose(startingSide==driverRelative.LEFT
                ?returnLocation==driverRelative.TRENCH?onTheFlySetpoints.TRENCH_LEFT_ALLIANCE:onTheFlySetpoints.RAMP_LEFT_ALLIANCE
                :returnLocation==driverRelative.TRENCH?onTheFlySetpoints.TRENCH_RIGHT_ALLIANCE:onTheFlySetpoints.RAMP_RIGHT_ALLIANCE));
    }

    public static void addAlignScorePosition(driverRelative scoreLocation){
        switch(scoreLocation){
            case LEFT: 
                AutoCommandHelper.addSetpoint(setpointToPose(onTheFlySetpoints.SCORE_LEFT));
                break;
            case RIGHT: 
                AutoCommandHelper.addSetpoint(setpointToPose(onTheFlySetpoints.SCORE_RIGHT));
                break;
            case CENTER: 
                AutoCommandHelper.addSetpoint(setpointToPose(onTheFlySetpoints.SCORE_CENTER));
                break;
            default: break;
        }
    }

    public static void addHumanPlayerCommand(){
        AutoCommandHelper.addSubsystemAction(intake.setIntakeIdleLength());
        AutoCommandHelper.addSetpoint(setpointToPose(onTheFlySetpoints.HUMAN_PLAYER));
        AutoCommandHelper.addPause(Seconds.of(5));
        addAlignScorePosition(driverRelative.RIGHT);
        addShootCommand();
    }

    public static void addClimbCommand(driverRelative climbSide){
        AutoCommandHelper.addSetpoint(setpointToPose(climbSide==driverRelative.LEFT?onTheFlySetpoints.CLIMB_LEFT_SETUP:onTheFlySetpoints.CLIMB_RIGHT_SETUP));
        AutoCommandHelper.addPause(Seconds.of(1));
        AutoCommandHelper.setMaxLinearVelocity(MetersPerSecond.of(1));
        AutoCommandHelper.addSetpoint(setpointToPose(climbSide==driverRelative.LEFT?onTheFlySetpoints.CLIMB_LEFT_SETUP:onTheFlySetpoints.CLIMB_RIGHT_SETUP));
        AutoCommandHelper.setMaxLinearVelocity(MetersPerSecond.of(4));
    }

    public static void addShootCommand(){
        // endPath sepparates different parallel command sequences
        AutoCommandHelper.endPath(0);
        AutoCommandHelper.addSubsystemAction(intake.setIntakeMaxLength().alongWith(shooter.startSpinner()).until(() -> Math.abs(shooter.getSpinnerClosedLoopError()) < 12 && shooter.getLeftSpinnerVelocity().magnitude() > 30));
        AutoCommandHelper.endPath(0);
        AutoCommandHelper.addSubsystemAction(indexer.runIndexer().withTimeout(6));
        AutoCommandHelper.addSubsystemAction(shooter.runKicker().withTimeout(6));
        AutoCommandHelper.addSubsystemAction(Commands.waitSeconds(4).andThen(intake.setIntakeMinLength()).withTimeout(6));
        AutoCommandHelper.endPath(0);
        AutoCommandHelper.addSubsystemAction(shooter.stopSpinner());
        AutoCommandHelper.addSubsystemAction(shooter.stopKicker());
        AutoCommandHelper.addSubsystemAction(intake.stopIntake());
    }

    public static void clearAll(){
        AutoCommandHelper.clearAll();
    }

    public static void removeLast(){
        AutoCommandHelper.undo();
    }

    public static void testAllExit(){
        addExitAlliance(driverRelative.TRENCH);
        addExitAlliance(driverRelative.RAMP);
    }
    public static void testAllReturn(){
        addReturnAlliance(driverRelative.TRENCH);
        addReturnAlliance(driverRelative.RAMP);
    }
    public static void testAllSweep(){
        addSweep(driverRelative.FAR, true);
        addSweep(driverRelative.CLOSE, true);
        addSweep(driverRelative.FAR, false);
        addSweep(driverRelative.CLOSE, false);
    }
    public static void testAllScore(){
        addAlignScorePosition(driverRelative.LEFT);
        addAlignScorePosition(driverRelative.CENTER);
        addAlignScorePosition(driverRelative.RIGHT);
    }
    public static void testAllClimb(){
        addClimbCommand(driverRelative.LEFT);
        addClimbCommand(driverRelative.RIGHT);
    }

    public static void testAllSequences(){
        setIsBlue(true);
        setStartingSide(driverRelative.LEFT);
        testAllExit();
        testAllSweep();
        testAllReturn();
        
        setStartingSide(driverRelative.RIGHT);
        testAllExit();
        testAllSweep();
        testAllReturn();

        testAllClimb();
        testAllScore();
        addHumanPlayerCommand();

        setIsBlue(false);
        setStartingSide(driverRelative.LEFT);
        testAllExit();
        testAllSweep();
        testAllReturn();

        setStartingSide(driverRelative.RIGHT);
        testAllExit();
        testAllSweep();
        testAllClimb();

        testAllClimb();
        testAllScore();
        addHumanPlayerCommand();
    
    }

    public static Pose2d setpointToPose(onTheFlySetpoints setpoint){
        return isBlue?setpoint.blueAlignmentPose:setpoint.redAlignmentPose;
    }
    
    public static Pose2d setpointToPose(onTheFlySetpoints setpoint, double targetRotationDegrees){
        Pose2d refferencePose = isBlue?setpoint.blueAlignmentPose:setpoint.redAlignmentPose;
        return new Pose2d(refferencePose.getX(), refferencePose.getY(), new Rotation2d(targetRotationDegrees));
    }
}
