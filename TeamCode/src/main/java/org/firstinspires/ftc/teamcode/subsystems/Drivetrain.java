package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.*;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.STICK_DEADBAND;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drivetrain {
    private Mecanum drivetrain;
    private PinpointLocalizer localizer;

    public void init(HardwareMap hardwareMap) {
        for (String name : new String[]{LEFT_FRONT, LEFT_BACK, RIGHT_FRONT, RIGHT_BACK}) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        drivetrain = Constants.createDrivetrain(hardwareMap);
        localizer = Constants.createLocalizer(hardwareMap);
    }

    public void update() {
        localizer.update();
    }

    public void drive(Gamepad gamepad, double power, boolean fieldCentric) {
        double forward = deadband(-gamepad.left_stick_y);
        double strafe = deadband(-gamepad.left_stick_x);
        double turn = deadband(-gamepad.right_stick_x);
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

    private double deadband(double input) {
        return Math.abs(input) > STICK_DEADBAND ? input : 0;
    }
}
