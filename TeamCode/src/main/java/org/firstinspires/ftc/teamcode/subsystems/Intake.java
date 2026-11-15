package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.*;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotorEx motor;
    private Servo feeder;

    public void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, INTAKE);
        motor.setPower(0);
        motor.setDirection(INTAKE_DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder = hardwareMap.get(Servo.class, FEEDER);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void feed(boolean feeding) {
        feeder.setPosition(feeding ? FEEDER_FEED : FEEDER_REST);
    }

    public void stop() {
        motor.setPower(0);
    }
}
