package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.commands.DriveCommands.drive;
import static org.firstinspires.ftc.teamcode.constants.AutoConstants.*;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndPose;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Disabled
@Autonomous(name = "Auto Template")
public class AutoTemplate extends OpMode {
    private Follower follower;
    private JoinedTelemetry joinedTele;

    @Override
    public void init() {
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(START_POSE);
    }

    @Override
    public void init_loop() {
        follower.update();
    }

    private Command routine() {
        return sequential(
                drive(follower, START_POSE, FORWARD_POSE),
                waitMs(250),
                drive(follower, FORWARD_POSE, TURN_POSE),
                drive(follower, TURN_POSE, CURVE_CONTROL, RETURN_POSE),
                waitMs(250),
                drive(follower, RETURN_POSE, PARK_POSE)
        );
    }

    @Override
    public void start() {
        follower.setPose(START_POSE);
        Scheduler.schedule(routine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        autoEndPose = follower.pose();
        joinedTele.addData("X", follower.pose().x());
        joinedTele.addData("Y", follower.pose().y());
        joinedTele.addData("Heading", Math.toDegrees(follower.pose().heading()));
        joinedTele.addData("Mode", follower.mode());
        joinedTele.update();
    }

    @Override
    public void stop() {
        autoEndPose = follower.pose();
        Scheduler.reset();
        follower.stop();
    }
}
