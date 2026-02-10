package frc.robot.commands;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helpers.TestHelpers;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class HubStatusAlertTest {
    private HubStatusAlert command;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        assertFalse(SimHooks.isTimingPaused());

        this.command = new HubStatusAlert();
    }

    @AfterEach
    public void cleanup() {
        // This method runs after each test to reset the scheduler state
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().run(); // Call run() to execute end() methods
    }

    @Test
    void initializeHidesAlerts() {
        command.emptyStatusAlert.set(true);
        command.badDataAlert.set(true);

        command.initialize();

        assertFalse(command.emptyStatusAlert.get());
        assertFalse(command.badDataAlert.get());
    }

    @Test
    void emptyStatusAlertShowsAfterTwoSeconds() {
        TestHelpers.setGameSpecificMessage("");

        command.initialize();

        SimHooks.stepTiming(1.8);
        command.execute();

        assertFalse(command.emptyStatusAlert.get());
        assertFalse(command.badDataAlert.get());

        SimHooks.stepTiming(0.3);
        command.execute();

        assertTrue(command.emptyStatusAlert.get());
        assertFalse(command.badDataAlert.get());
    }
}
