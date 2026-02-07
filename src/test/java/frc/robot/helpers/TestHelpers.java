package frc.robot.helpers;

import bot.den.foxflow.RobotState;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class TestHelpers {
    public static void setDriverStationState(RobotState state) {
        switch(state) {
            case DISABLED -> {
                DriverStationSim.setEnabled(false);
                DriverStationSim.setAutonomous(false);
                DriverStationSim.setTest(false);
            }
            case AUTO -> {
                DriverStationSim.setAutonomous(true);
                DriverStationSim.setTest(false);
                DriverStationSim.setEnabled(true);
                DriverStationSim.setDsAttached(true);
            }
            case TELEOP -> {
                DriverStationSim.setAutonomous(false);
                DriverStationSim.setTest(false);
                DriverStationSim.setEnabled(true);
                DriverStationSim.setDsAttached(true);
            }
            case TEST -> {
                DriverStationSim.setTest(true);
                DriverStationSim.setAutonomous(false);
                DriverStationSim.setEnabled(true);
                DriverStationSim.setDsAttached(true);
            }
        }

        DriverStationSim.notifyNewData();
    }
}