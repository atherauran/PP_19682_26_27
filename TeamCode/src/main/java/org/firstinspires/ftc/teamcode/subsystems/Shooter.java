package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.*;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
    private DcMotorEx leftShooter, rightShooter;
    private Servo hood;
    private double targetVelocity;

    public void init(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, LEFT_SHOOTER);
        rightShooter = hardwareMap.get(DcMotorEx.class, RIGHT_SHOOTER);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        leftShooter.setDirection(LEFT_SHOOTER_DIRECTION);
        rightShooter.setDirection(RIGHT_SHOOTER_DIRECTION);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // leftShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        // rightShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hood = hardwareMap.get(Servo.class, HOOD);
    }

    public void setVelocity(double velocity) {
        targetVelocity = Range.clip(velocity, 0, SHOOTER_MAX_VELOCITY);
        if (targetVelocity == 0) {
            stop();
        } else {
            leftShooter.setVelocity(targetVelocity);
            rightShooter.setVelocity(targetVelocity);
        }
    }

    public void setHood(double position) {
        hood.setPosition(Range.clip(position, HOOD_MIN, HOOD_MAX));
    }

    public double velocityForDistance(double distance) {
        return interpolate(SHOT_DISTANCES, SHOT_VELOCITIES, distance);
    }

    public double hoodForDistance(double distance) {
        return interpolate(SHOT_DISTANCES, SHOT_HOODS, distance);
    }

    private static double interpolate(double[] xs, double[] ys, double x) {
        if (x <= xs[0]) return ys[0];
        for (int i = 1; i < xs.length; i++) {
            if (x <= xs[i]) {
                double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
                return ys[i - 1] + t * (ys[i] - ys[i - 1]);
            }
        }
        return ys[ys.length - 1];
    }

    public double getHoodPosition() {
        return hood.getPosition();
    }

    public double getLeftVelocity() {
        return leftShooter.getVelocity();
    }

    public double getRightVelocity() {
        return rightShooter.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public boolean isReady() {
        return targetVelocity > 0
                && Math.abs(getLeftVelocity() - targetVelocity) <= SHOOTER_VELOCITY_TOLERANCE
                && Math.abs(getRightVelocity() - targetVelocity) <= SHOOTER_VELOCITY_TOLERANCE;
    }

    public void stop() {
        targetVelocity = 0;
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }
}
