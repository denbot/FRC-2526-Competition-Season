package frc.robot.subsystems.vision;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.SocketTimeoutException;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.Limelights.Limelight;

public class LimelightIOReal implements LimelightIO{

    private final Map<String, Boolean> limelightCache = new HashMap<>();
    private final Map<String, Long> limelightLastCheckTimer = new HashMap<>();

    private boolean isLimelightConnected(Limelight limelight){
        // We only want to check every so often, instead of every 20ms. Every second is sufficient.
        long fpgaTime = RobotController.getFPGATime();
        if (limelightLastCheckTimer.containsKey(limelight.name)) {
            long elapsedTime = fpgaTime - limelightLastCheckTimer.get(limelight.name);

            if (elapsedTime < 1_000_000) { // 1 second in microseconds
                return limelightCache.getOrDefault(limelight.name, false);
            }
            limelightLastCheckTimer.put(limelight.name, fpgaTime);
        } else {
            // Okay, so we haven't been checked ever. Let's see if anyone else got checked this loop
            // Note that "this loop" is approximate, we just verify it wasn't in the last 20 ms.
            for (Map.Entry<String, Long> lastChecked : limelightLastCheckTimer.entrySet()) {
                long elapsedTime = fpgaTime - lastChecked.getValue();
                if (elapsedTime < 20) {
                    return false; // We haven't been checked and another camera got checked this loop
                }
            }
        }

        boolean limelightFound = isLimelightFound(limelight);

        limelightCache.put(limelight.name, limelightFound);
        limelightLastCheckTimer.put(limelight.name, fpgaTime);

        return limelightFound;
    }

    private boolean isLimelightFound(Limelight limelight) {
        String url = String.format("http://%s/", limelight.ip);

        try {
            HttpURLConnection connection = (HttpURLConnection) new URL(url).openConnection();
            connection.setConnectTimeout(5);
            connection.setReadTimeout(5);
            connection.setRequestMethod("HEAD");
            int responseCode = connection.getResponseCode();

            return responseCode == 200;
        } catch (SocketTimeoutException e) {
            return false;
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs){
        
        // If each is connected
        inputs.backLeftConnected = isLimelightConnected(Limelight.BACK_LEFT);
        inputs.backRightConnected = isLimelightConnected(Limelight.BACK_RIGHT);
        inputs.frontConnected = isLimelightConnected(Limelight.FRONT);
        inputs.allConnected = inputs.backLeftConnected&&inputs.backRightConnected&&inputs.frontConnected;

        // Tag count
        // pre-define to check for null cases
        PoseEstimate poseEstimate1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Limelight.BACK_LEFT.name);
        PoseEstimate poseEstimate2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Limelight.BACK_RIGHT.name);
        PoseEstimate poseEstimate3 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Limelight.FRONT.name);

        inputs.backLeftTagCount = poseEstimate1 == null ? 0 : poseEstimate1.tagCount;
        inputs.backRightTagCount = poseEstimate2 == null ? 0 : poseEstimate2.tagCount;
        inputs.frontTagCount = poseEstimate3 == null ? 0 : poseEstimate3.tagCount;

        inputs.totalTagCount = inputs.backLeftTagCount + inputs.backRightTagCount + inputs.frontTagCount;

    }

    public void getAllPoseEstimate(Drive drive){
        getSinglePoseEstimate(drive, Limelight.BACK_LEFT);
        getSinglePoseEstimate(drive, Limelight.BACK_RIGHT);
        getSinglePoseEstimate(drive, Limelight.FRONT);
    }

    public void getSinglePoseEstimate(Drive drive, Limelight limelight){
        LimelightHelpers.PoseEstimate poseEstimate = 
        LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name);

        if(poseEstimate == null || poseEstimate.tagCount == 0) return;

        if (poseEstimate.tagCount == 1 && 
        poseEstimate.rawFiducials.length == 1 &&
        (poseEstimate.rawFiducials[0].ambiguity > .7 || 
        poseEstimate.rawFiducials[0].distToCamera > 3)) return;
        
        drive.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, limelight.visionMatrix);
    }
}
