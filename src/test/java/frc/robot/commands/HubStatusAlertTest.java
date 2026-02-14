package frc.robot.commands;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helpers.TestHelpers;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.ArrayList;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class HubStatusAlertTest {
    private HubStatusAlert command;
    private final Random random = new Random();

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));

        SimHooks.pauseTiming();

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

    @Test
    void badDataAlertShowsAfterTwoSeconds() {
        int[] randomInts = {random.nextInt(257),random.nextInt(257),random.nextInt(257),random.nextInt(257),random.nextInt(257)};
        ArrayList<Character> randomChars = new ArrayList<>();
        for (int randomInt : randomInts) {
            randomChars.add((char) randomInt);
        }
        String badData = randomChars.toString();
        TestHelpers.setGameSpecificMessage(badData);

        command.initialize();

        SimHooks.stepTiming(1.8);
        command.execute();

        assertFalse(command.emptyStatusAlert.get());
        assertFalse(command.badDataAlert.get());

        SimHooks.stepTiming(0.3);
        command.execute();

        assertFalse(command.emptyStatusAlert.get());
        assertTrue(command.badDataAlert.get());
    }

    @ParameterizedTest
    @ValueSource(strings = {"R", "B"})
    void noAlertsShowOnGoodData(String data) {
        TestHelpers.setGameSpecificMessage(data);

        command.initialize();

        SimHooks.stepTiming(2.1);

        command.execute();

        assertFalse(command.emptyStatusAlert.get());
        assertFalse(command.badDataAlert.get());
    }

    @ParameterizedTest
    @ValueSource(strings = {"R", "B"})
    void emptyStatusAlertIsRemovedAfterGoodDataSent(String data) {
        TestHelpers.setGameSpecificMessage("");

        command.initialize();

        SimHooks.stepTiming(2.1);

        command.execute();

        assertTrue(command.emptyStatusAlert.get());
        assertFalse(command.badDataAlert.get());

        TestHelpers.setGameSpecificMessage(data);

        command.execute();

        assertFalse(command.emptyStatusAlert.get());
        assertFalse(command.badDataAlert.get());
    }

    @ParameterizedTest
    @ValueSource(strings = {"R", "B"})
    void badDataAlertIsRemovedAfterGoodDataSent(String data) {
        int[] randomInts = {random.nextInt(257),random.nextInt(257),random.nextInt(257),random.nextInt(257),random.nextInt(257)};
        ArrayList<Character> randomChars = new ArrayList<>();
        for (int randomInt : randomInts) {
            randomChars.add((char) randomInt);
        }
        String badData = randomChars.toString();
        TestHelpers.setGameSpecificMessage(badData);

        command.initialize();

        SimHooks.stepTiming(2.1);

        command.execute();

        assertFalse(command.emptyStatusAlert.get());
        assertTrue(command.badDataAlert.get());

        TestHelpers.setGameSpecificMessage(data);

        command.execute();

        assertFalse(command.emptyStatusAlert.get());
        assertFalse(command.badDataAlert.get());
    }

}
