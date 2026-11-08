package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public final Drivetrain drivetrain = new Drivetrain();

    public void init(HardwareMap hardwareMap) {
        drivetrain.init(hardwareMap);
    }

    public void update() {
        drivetrain.update();
    }

    public void stop() {
        drivetrain.stop();
    }
}
