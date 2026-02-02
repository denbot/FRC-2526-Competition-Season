package frc.robot.subsystems.auto;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShuffleBoardInputs {
    private boolean LeftOrRight = false;
    private boolean ExitTrenchOrRamp = false;
    private boolean EdgeOrCenter = false;
    private boolean ReturnTrenchOrRamp = false;
    private boolean ClimbLeftOrRight = false;

    public ShuffleBoardInputs() {
    
    SmartDashboard.putBoolean("LeftOrRightSwitch", LeftOrRight);
    SmartDashboard.putBoolean("ExitTrenchOrRampSwitch", ExitTrenchOrRamp);
    SmartDashboard.putBoolean("EdgeOrCenterSwitch", EdgeOrCenter);
    SmartDashboard.putBoolean("ReturnTrenchOrRampSwitch", ReturnTrenchOrRamp);
    SmartDashboard.putBoolean("ClimbLeftOrRight", ClimbLeftOrRight);

    /* All the basic auto paths we can do:
     * 1) Starts on left side, exits through trench, picks up on the edge, returns through trench 
     * 2) Starts on left side, exits through trench, picks up on the edge, returns through ramp
     * 3) Starts on left side, exits through trench, picks up in the center, returns through trench
     * 4) Starts on left side, exits through trench, picks up in the center, returns through ramp
     * 5) Starts on left side, exits through ramp, picks up on the edge, returns through trench
     * 6) Starts on left side, exits through ramp, picks up on the edge, returns through ramp
     * 7) Starts on left side, exits through ramp, picks up in the center, returns through trench
     * 8) Starts on left side, exits through ramp, picks up in the center, returns through ramp
     * 9) Starts on right side, exits through trench, picks up on the edge, returns through trench
     * 10) Starts on right side, exits through trench, picks up on the edge, returns through ramp
     * 11) Starts on the right side, exits through trench, picks up in the center, returns through trench
     * 12) Starts on the right side, exits through trench, picks up in the center, returns through ramp
     * 13) Starts on the right side, exits through the ramp, picks up on the edge, returns through trench
     * 14) Starts on the right side, exits through the ramp, picks up on the edge, returns through ramp
     * 15) Starts on the right side, exits through the ramp, picks up in the center, returns through trench
     * 16) Starts on the right side, exits through the ramp, picks up in the center, returns through ramp
     * 
     * LeftOrRight
     *  true = right
     *  false = left
     * ExitTrenchOrRamp
     *  true = ramp
     *  false = trench
     * EdgeOrCenter
     *  true = center
     *  false = edge
     * ReturnTrenchOrRamp
     *  true = ramp
     *  false = trench
    */

    if (LeftOrRight == false && ExitTrenchOrRamp == false && EdgeOrCenter == false && ReturnTrenchOrRamp == false) {
       //return case 1
    } 
    if (LeftOrRight == false && ExitTrenchOrRamp == false && EdgeOrCenter == false && ReturnTrenchOrRamp == true) {
        //return case 2
    }
    if (LeftOrRight == false && ExitTrenchOrRamp == false && EdgeOrCenter == true && ReturnTrenchOrRamp == false) {
        //return case 3
    }
    if (LeftOrRight == false && ExitTrenchOrRamp == false && EdgeOrCenter == true && ReturnTrenchOrRamp == true) {
        //return case 4
    }
    if (LeftOrRight == false && ExitTrenchOrRamp == true && EdgeOrCenter == false && ReturnTrenchOrRamp == false) {
        //return case 5
    }
    if (LeftOrRight == false && ExitTrenchOrRamp == true && EdgeOrCenter == false && ReturnTrenchOrRamp == true) {
        //return case 6
    }
    if (LeftOrRight == false && ExitTrenchOrRamp == true && EdgeOrCenter == true && ReturnTrenchOrRamp == false) {
        //return case 7
    }
    if (LeftOrRight == false && ExitTrenchOrRamp == true && EdgeOrCenter == true && ReturnTrenchOrRamp == true) {
        //return case 8
    }
    if (LeftOrRight == true && ExitTrenchOrRamp == false && EdgeOrCenter == false && ReturnTrenchOrRamp == false) {
       //return case 9
    } 
    if (LeftOrRight == true && ExitTrenchOrRamp == false && EdgeOrCenter == false && ReturnTrenchOrRamp == true) {
        //return case 10
    }
    if (LeftOrRight == true && ExitTrenchOrRamp == false && EdgeOrCenter == true && ReturnTrenchOrRamp == false) {
        //return case 11
    }
    if (LeftOrRight == true && ExitTrenchOrRamp == false && EdgeOrCenter == true && ReturnTrenchOrRamp == true) {
        //return case 12
    }
    if (LeftOrRight == true && ExitTrenchOrRamp == true && EdgeOrCenter == false && ReturnTrenchOrRamp == false) {
        //return case 13
    }
    if (LeftOrRight == true && ExitTrenchOrRamp == true && EdgeOrCenter == false && ReturnTrenchOrRamp == true) {
        //return case 14
    }
    if (LeftOrRight == true && ExitTrenchOrRamp == true && EdgeOrCenter == true && ReturnTrenchOrRamp == false) {
        //return case 15
    }
    if (LeftOrRight == true && ExitTrenchOrRamp == true && EdgeOrCenter == true && ReturnTrenchOrRamp == true) {
        //return case 16
    }

}
}

