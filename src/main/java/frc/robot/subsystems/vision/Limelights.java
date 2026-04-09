package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class Limelights extends SubsystemBase{

    public final LimelightIO io;
    private final LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();
    private final Drive drive;

    public enum Limelight {
        BACK_LEFT("limelight-left", LimelightConstants.backLeftLimelightIP),
        BACK_RIGHT("limelight-right", LimelightConstants.backRightLimelightIP),
        FRONT("limelight-front", LimelightConstants.frontLimelightIP);

        public final String name;
        public final String ip;
        public final Matrix<N3, N1> visionMatrixRejectsRotation = new Matrix<>(Nat.N3(), Nat.N1());
        public final Matrix<N3, N1> visionMatrixAcceptsRotation = new Matrix<>(Nat.N3(), Nat.N1());
        private final int[] idsToUse = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28};

        Limelight(String name, String ip) {
            this.name = name;
            this.ip = ip;
            this.visionMatrixRejectsRotation.fill(0.5);
            this.visionMatrixRejectsRotation.set(2, 0, 10000000);
            this.visionMatrixAcceptsRotation.fill(0.5);
            LimelightHelpers.SetFiducialIDFiltersOverride(name, idsToUse);
        }
    }

    @Override
    public void periodic(){
        // Log key variables
        io.updateInputs(inputs);
    }

    public Limelights(LimelightIO io, Drive drive){
        this.io = io;
        this.drive = drive;
    }    

    public void getAllPoseEstimate(){
        this.io.getAllPoseEstimate(this.drive);
    }

    public boolean getAllConnected(){
        return inputs.allConnected;
    }
    
    public boolean getBackLeftConnected(){
        return this.inputs.backLeftConnected;
    }
    
    public boolean getBackRightConnected(){
        return this.inputs.backRightConnected;
    }
    
    public boolean getFrontConnected(){
        return this.inputs.frontConnected;
    }

    public int getBackRightTags(){
        return inputs.backLeftTagCount;
    }
    
    public int getBackLeftTags(){
        return inputs.backRightTagCount;
    }
    
    public int getFrontTags(){
        return inputs.frontTagCount;
    }
    
    public int getTotalTagCount(){
        return inputs.totalTagCount;
    }
}