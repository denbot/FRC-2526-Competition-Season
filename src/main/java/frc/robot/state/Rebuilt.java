package frc.robot.state;

import bot.den.foxflow.RobotState;
import bot.den.foxflow.StateMachine;

@StateMachine
public record Rebuilt(
    RobotState robotState,
    MatchState matchState,
    HubState hubState
) {}
