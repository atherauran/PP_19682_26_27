package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.commands.IntakeCommands.feed;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.*;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Basic TeleOp")
public class BasicTeleOp extends LinearOpMode {
    private final Robot robot = new Robot();
    private JoinedTelemetry joinedTele;
    private boolean fieldCentric = false;
    private boolean shooterOn = false;
    private Command feedCommand;

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        robot.init(hardwareMap);
        try {
            joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
            robot.drivetrain.setPose(Pose.zero());
            feedCommand = feed(robot.intake);
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
            robot.drivetrain.setPose(robot.drivetrain.getPose().withHeading(0));
        }
        robot.drivetrain.drive(gamepad1, gamepad1.left_stick_button ? SLOW_DRIVE_POWER : DRIVE_POWER, fieldCentric);
    }

    private void updateMechanisms() {
        boolean collect = gamepad1.right_trigger > 0.1;
        boolean eject = gamepad1.left_trigger > 0.1;
        boolean stopped = gamepad1.b || gamepad2.b;
        boolean feedPressed = gamepad1.rightBumperWasPressed();

        if (gamepad1.leftBumperWasPressed()) shooterOn = !shooterOn;
        if (collect || eject || stopped) shooterOn = false;

        if (gamepad2.dpadUpWasPressed()) SHOOTER_VELOCITY += SHOOTER_VELOCITY_STEP;
        if (gamepad2.dpadDownWasPressed()) SHOOTER_VELOCITY -= SHOOTER_VELOCITY_STEP;
        SHOOTER_VELOCITY = Range.clip(SHOOTER_VELOCITY, 0, SHOOTER_MAX_VELOCITY);
        if (stopped) robot.shooter.stop();
        else robot.shooter.setVelocity(shooterOn ? SHOOTER_VELOCITY : SHOOTER_IDLE_VELOCITY);

        double hoodPosition = robot.shooter.getHoodPosition();
        if (gamepad2.yWasPressed()) hoodPosition += HOOD_STEP;
        if (gamepad2.aWasPressed()) hoodPosition -= HOOD_STEP;
        if (!stopped) robot.shooter.setHood(hoodPosition);

        boolean resetTurret = gamepad2.startWasPressed();
        if (resetTurret) robot.turret.resetEncoder();
        robot.turret.rotate(stopped || resetTurret ? 0 : -gamepad2.right_stick_x);

        if (!shooterOn || SHOOTER_VELOCITY == 0) {
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
        Pose pose = robot.drivetrain.getPose();
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
        joinedTele.addData("Turret limits", TURRET_LIMITS_ENABLED ? "Enabled" : "OFF - watch cables");
        joinedTele.addData("Auto aim", "Disabled");
        joinedTele.update();
    }
}
