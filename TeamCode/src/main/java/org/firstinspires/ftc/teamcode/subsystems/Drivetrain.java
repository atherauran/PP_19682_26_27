package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivetrain {
    private Mecanum drivetrain;
    private PinpointLocalizer localizer;

    public void init(HardwareMap hardwareMap) {
        drivetrain = Constants.createDrivetrain(hardwareMap);
        localizer = Constants.createLocalizer(hardwareMap);
    }

    public void update() {
        localizer.update();
    }

    public void drive(Gamepad gamepad, double power, boolean fieldCentric) {
        double forward = -gamepad.left_stick_y;
        double strafe = -gamepad.left_stick_x;
        double turn = -gamepad.right_stick_x;
        DrivePowers powers = fieldCentric
                ? ManualDrive.fieldCentric(forward, strafe, turn, getPose().heading())
                : new DrivePowers(forward, strafe, turn);
        double scale = power / Math.max(1.0,
                Math.abs(powers.forward()) + Math.abs(powers.strafe()) + Math.abs(powers.turn()));
        drivetrain.drive(new DrivePowers(powers.forward() * scale,
                powers.strafe() * scale, powers.turn() * scale), true);
    }

    public Pose getPose() {
        return localizer.state().pose();
    }

    public void setPose(Pose pose) {
        localizer.setPose(pose);
    }

    public void stop() {
        drivetrain.stop();
    }
}
