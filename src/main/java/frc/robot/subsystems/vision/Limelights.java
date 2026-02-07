package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.RobotController;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.SocketTimeoutException;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;

public enum Limelights {
    LEFT("limelight-left", "10.95.86.11"),
    RIGHT("limelight-right", "10.95.86.13"),
    REAR("limelight-rear", "10.95.86.12");

    private static final Map<String, Boolean> limelightCache = new HashMap<>();
    private static final Map<String, Long> limelightLastCheckTimer = new HashMap<>();

    public final String name;
    private final String ip;

    Limelights(String name, String ip) {
        this.name = name;
        this.ip = ip;
    }

    public boolean isConnected() {
        // We only want to check every so often, instead of every 20ms. Every second is sufficient.
        long fpgaTime = RobotController.getFPGATime();
        if (limelightLastCheckTimer.containsKey(name)) {
            long elapsedTime = fpgaTime - limelightLastCheckTimer.get(name);

            if (elapsedTime < 1_000_000) { // 1 second in microseconds
                return limelightCache.getOrDefault(name, false);
            }
            limelightLastCheckTimer.put(name, fpgaTime);
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

        boolean limelightFound = isLimelightFound();

        limelightCache.put(name, limelightFound);
        limelightLastCheckTimer.put(name, fpgaTime);

        return limelightFound;
    }

    private boolean isLimelightFound() {
        String url = String.format("http://%s/", ip);

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
}