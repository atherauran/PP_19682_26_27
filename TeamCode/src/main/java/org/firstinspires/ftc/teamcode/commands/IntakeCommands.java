package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.FEEDER_PULSE_MS;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeCommands {
    public static Command feed(Intake intake) {
        return sequential(
                instant(() -> intake.feed(true)),
                waitMs(FEEDER_PULSE_MS),
                instant(() -> intake.feed(false))
        );
    }
}
