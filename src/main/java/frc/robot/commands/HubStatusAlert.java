package frc.robot.commands;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Seconds;

public class HubStatusAlert extends Command {
    public final Alert emptyStatusAlert;
    public final Alert badDataAlert;
    private final Timer timer;
    private boolean goodData;

    public HubStatusAlert() {
        emptyStatusAlert = new Alert("Empty Status", Alert.AlertType.kError);
        badDataAlert = new Alert("Bad Data", Alert.AlertType.kError);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        emptyStatusAlert.set(false);
        badDataAlert.set(false);
        timer.restart();
        goodData = false;
    }

    @Override
    public void execute() {
        String gameSpecificMessage = DriverStation.getGameSpecificMessage();
        if (!gameSpecificMessage.isEmpty()) {
            if (gameSpecificMessage.equals("R") || gameSpecificMessage.equals("B")) {
                emptyStatusAlert.set(false);
                badDataAlert.set(false);
                goodData = true;
            } else {
                badDataAlert.set(true);
            }
        } else if (timer.hasElapsed(Seconds.of(2))) {
            emptyStatusAlert.set(true);
        }
    }

    @Override
    public boolean isFinished() {
        return goodData;
    }
}
