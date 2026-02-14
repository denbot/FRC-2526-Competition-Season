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
        BACK_LEFT("limelight-back-left", LimelightConstants.backLeftLimelightIP),
        BACK_RIGHT("limelight-back-right", LimelightConstants.backRightLimelightIP),
        FRONT("limelight-front", LimelightConstants.frontLimelightIP);

        public final String name;
        public final String ip;
        public final Matrix<N3, N1> visionMatrix = new Matrix<>(Nat.N3(), Nat.N1());

        Limelight(String name, String ip) {
            this.name = name;
            this.ip = ip;
            this.visionMatrix.fill(0.5);
            this.visionMatrix.set(2, 0, 1);
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
    
    public int geTotalTagCount(){
        return inputs.totalTagCount;
    }
}