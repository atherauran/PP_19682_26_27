package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public final Drivetrain drivetrain = new Drivetrain();
    public final Intake intake = new Intake();
    public final Shooter shooter = new Shooter();
    public final Turret turret = new Turret();

    public void init(HardwareMap hardwareMap) {
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);
    }

    public void update() {
        drivetrain.update();
    }

    public void stop() {
        drivetrain.stop();
        intake.stop();
        shooter.stop();
        turret.stop();
    }
}
