package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.commands.IntakeCommands.feed;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.*;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public abstract class TeleOpBase extends LinearOpMode {
    private final Robot robot = new Robot();
    private final AutoAim autoAim = new AutoAim();
    private JoinedTelemetry joinedTele;
    private boolean fieldCentric = false;
    private boolean shooterOn = false;
    private boolean autoAimOn = false;
    private boolean movingShoot = false;
    private boolean farCellUp;
    private double turretCorrection = 0;
    private double velocityCorrection = 0;
    private double hoodCorrection = 0;
    private Pose pose;
    private Velocity velocity;
    private double turretX, turretY, targetX, targetY, distance;
    private AutoAim.TurretCommand aimCommand = new AutoAim.TurretCommand();
    private Command feedCommand;

    protected abstract String getAllianceName();

    protected abstract Pose getCell(boolean farCell);

    protected abstract boolean farCellStartsUp();

    protected abstract Pose getRelocalizePose();

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        robot.init(hardwareMap);
        autoAim.init(robot.turret, hardwareMap);
        try {
            joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
            robot.drivetrain.setPose(autoEndPose);
            farCellUp = farCellStartsUp();
            feedCommand = feed(robot.intake);
            joinedTele.addData("Alliance", getAllianceName());
            joinedTele.addLine("Center turret before INIT.");
            joinedTele.addData("Turret limits", TURRET_LIMITS_ENABLED ? "Enabled" : "OFF - watch cables");
            joinedTele.update();

            waitForStart();
            if (isStopRequested()) return;

            robot.shooter.setHood(HOOD_START);
            robot.intake.feed(false);
            while (opModeIsActive()) {
                robot.update();
                updateDrive();
                updateTarget();
                updateMechanisms();
                updateTelemetry();
            }
        } finally {
            Scheduler.reset();
            robot.stop();
        }
    }

    private void updateDrive() {
        if (gamepad1.optionsWasPressed()) {
            fieldCentric = !fieldCentric;
        }
        if (gamepad1.startWasPressed()) {
            robot.drivetrain.setPose(getRelocalizePose());
        }
        robot.drivetrain.drive(gamepad1, gamepad1.left_stick_button ? SLOW_DRIVE_POWER : DRIVE_POWER,
                fieldCentric, getRelocalizePose().heading());
    }

    private void updateTarget() {
        if (gamepad2.xWasPressed()) farCellUp = !farCellUp;
        if (gamepad1.yWasPressed()) movingShoot = !movingShoot;

        pose = robot.drivetrain.getPose();
        velocity = robot.drivetrain.getVelocity();
        double h = pose.heading();
        turretX = pose.x() + TURRET_OFFSET_X * Math.cos(h) - TURRET_OFFSET_Y * Math.sin(h);
        turretY = pose.y() + TURRET_OFFSET_X * Math.sin(h) + TURRET_OFFSET_Y * Math.cos(h);

        Pose cell = getCell(farCellUp);
        targetX = cell.x();
        targetY = cell.y();
        if (movingShoot) {
            double at = Math.hypot(targetX - turretX, targetY - turretY) * SOTM_TIME_PER_INCH + SOTM_BASE_TIME;
            targetX -= at * velocity.vx;
            targetY -= at * velocity.vy;
        }
        distance = Math.hypot(targetX - turretX, targetY - turretY);
    }

    private void updateMechanisms() {
        boolean collect = gamepad1.right_trigger > 0.1;
        boolean eject = gamepad1.left_trigger > 0.1;
        boolean stopped = gamepad1.b || gamepad2.b;
        boolean feedPressed = gamepad1.rightBumperWasPressed();

        if (gamepad1.xWasPressed()) autoAimOn = !autoAimOn;
        boolean aiming = autoAimOn && AUTO_AIM_ENABLED;

        if (gamepad1.leftBumperWasPressed()) shooterOn = !shooterOn;
        if (collect || eject || stopped) shooterOn = false;

        double velocityStep = 0, hoodStep = 0;
        if (gamepad2.dpadUpWasPressed()) velocityStep += SHOOTER_VELOCITY_STEP;
        if (gamepad2.dpadDownWasPressed()) velocityStep -= SHOOTER_VELOCITY_STEP;
        if (gamepad2.yWasPressed()) hoodStep += HOOD_STEP;
        if (gamepad2.aWasPressed()) hoodStep -= HOOD_STEP;
        if (gamepad2.dpadLeftWasPressed()) turretCorrection += 1;
        if (gamepad2.dpadRightWasPressed()) turretCorrection -= 1;

        double shotVelocity, hoodPosition;
        if (aiming) {
            velocityCorrection += velocityStep;
            hoodCorrection += hoodStep;
            shotVelocity = Range.clip(robot.shooter.velocityForDistance(distance) + velocityCorrection,
                    0, SHOOTER_MAX_VELOCITY);
            hoodPosition = robot.shooter.hoodForDistance(distance) + hoodCorrection;
        } else {
            SHOOTER_VELOCITY = Range.clip(SHOOTER_VELOCITY + velocityStep, 0, SHOOTER_MAX_VELOCITY);
            shotVelocity = SHOOTER_VELOCITY;
            hoodPosition = robot.shooter.getHoodPosition() + hoodStep;
        }
        if (stopped) robot.shooter.stop();
        else robot.shooter.setVelocity(shooterOn ? shotVelocity : SHOOTER_IDLE_VELOCITY);
        if (!stopped) robot.shooter.setHood(hoodPosition);

        boolean resetTurret = gamepad2.startWasPressed();
        if (resetTurret) robot.turret.resetEncoder();
        if (aiming && !stopped && !resetTurret) {
            aimCommand = autoAim.update(turretX, turretY, Math.toDegrees(pose.heading()),
                    Math.toDegrees(velocity.omega), targetX, targetY, turretCorrection);
        } else {
            autoAim.reset();
            robot.turret.rotate(stopped || resetTurret ? 0 : -gamepad2.right_stick_x);
        }

        if (!shooterOn || shotVelocity == 0) {
            Scheduler.cancel(feedCommand);
            robot.intake.feed(false);
        } else if (feedPressed && !Scheduler.isScheduled(feedCommand)) {
            feedCommand = feed(robot.intake);
            Scheduler.schedule(feedCommand);
        }
        Scheduler.execute();

        double intakePower = 0;
        if (!stopped) {
            if (eject) intakePower = -gamepad1.left_trigger * INTAKE_POWER;
            else if (collect) intakePower = gamepad1.right_trigger * INTAKE_POWER;
            else if (Scheduler.isScheduled(feedCommand)) intakePower = FEED_INTAKE_POWER;
        }
        robot.intake.setPower(intakePower);
    }

    private void updateTelemetry() {
        boolean aiming = autoAimOn && AUTO_AIM_ENABLED;
        joinedTele.addData("Alliance", getAllianceName());
        joinedTele.addData("Drive", fieldCentric ? "Field centric" : "Robot centric");
        joinedTele.addData("X (in)", pose.x());
        joinedTele.addData("Y (in)", pose.y());
        joinedTele.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        joinedTele.addData("Shooter", shooterOn ? "SHOOTING"
                : robot.shooter.getTargetVelocity() > 0 ? "IDLE" : "STOPPED");
        joinedTele.addData("Target velocity (ticks/s)", robot.shooter.getTargetVelocity());
        joinedTele.addData("Left velocity (ticks/s)", robot.shooter.getLeftVelocity());
        joinedTele.addData("Right velocity (ticks/s)", robot.shooter.getRightVelocity());
        joinedTele.addData("At speed", shooterOn && robot.shooter.isReady());
        joinedTele.addData("Feeding", Scheduler.isScheduled(feedCommand));
        joinedTele.addData("Hood", robot.shooter.getHoodPosition());
        joinedTele.addData("Turret (ticks)", robot.turret.getPosition());
        joinedTele.addData("Turret (deg)", robot.turret.getDegrees());
        joinedTele.addData("Turret limits", TURRET_LIMITS_ENABLED ? "Enabled" : "OFF - watch cables");
        joinedTele.addData("Auto aim", !AUTO_AIM_ENABLED ? "Disabled" : autoAimOn ? "ON" : "OFF");
        joinedTele.addData("Target CELL", farCellUp ? "Far" : "Audience");
        joinedTele.addData("Moving shot", movingShoot);
        joinedTele.addData("Distance (in)", distance);
        joinedTele.addData("Turret correction (deg)", turretCorrection);
        if (aiming) {
            joinedTele.addData("Aim has target", aimCommand.hasTarget);
            joinedTele.addData("Aim target (deg)", aimCommand.targetTurretAngle);
            joinedTele.addData("Aim error (deg)", aimCommand.aimError);
            joinedTele.addData("Velocity correction (ticks/s)", velocityCorrection);
            joinedTele.addData("Hood correction", hoodCorrection);
        }
        joinedTele.update();
    }
}
