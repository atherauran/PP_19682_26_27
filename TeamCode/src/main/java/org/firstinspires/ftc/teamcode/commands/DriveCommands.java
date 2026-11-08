package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

public class DriveCommands {
    public static Command drive(Follower follower, Pose from, Pose to) {
        return follow(follower, line(from, to).linear(from, to));
    }

    public static Command drive(Follower follower, Pose from, Pose control, Pose to) {
        return follow(follower, curve(from, control, to).linear(from, to));
    }
}
