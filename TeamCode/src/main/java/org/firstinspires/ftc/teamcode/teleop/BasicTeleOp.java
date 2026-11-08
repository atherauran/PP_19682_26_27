package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndPose;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Basic TeleOp")
public class BasicTeleOp extends LinearOpMode {
    private final Robot robot = new Robot();
    private JoinedTelemetry joinedTele;
    private boolean fieldCentric = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        waitForStart();
        robot.drivetrain.setPose(autoEndPose);

        while (opModeIsActive()) {
            robot.update();
            if (gamepad1.optionsWasPressed()) fieldCentric = !fieldCentric;

            robot.drivetrain.drive(gamepad1, 1.0, fieldCentric);

            Pose pose = robot.drivetrain.getPose();
            joinedTele.addData("Drive", fieldCentric ? "Field centric" : "Robot centric");
            joinedTele.addData("X", pose.x());
            joinedTele.addData("Y", pose.y());
            joinedTele.addData("Heading", Math.toDegrees(pose.heading()));
            joinedTele.update();
        }

        robot.stop();
    }
}
