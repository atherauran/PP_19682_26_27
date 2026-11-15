package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.*;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotorEx motor;

    public void init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, TURRET);
        motor.setDirection(TURRET_DIRECTION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoder();
    }

    public void rotate(double input) {
        double power = Math.abs(input) > STICK_DEADBAND
                ? input * TURRET_MANUAL_POWER : 0;
        if (TURRET_LIMITS_ENABLED) {
            int position = getPosition();
            if ((position <= TURRET_MIN_TICKS && power < 0)
                    || (position >= TURRET_MAX_TICKS && power > 0)) {
                power = 0;
            }
        }
        motor.setPower(power);
    }

    // This establishes a reference only; it does not move or home the turret.
    public void resetEncoder() {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public void stop() {
        motor.setPower(0);
    }
}
