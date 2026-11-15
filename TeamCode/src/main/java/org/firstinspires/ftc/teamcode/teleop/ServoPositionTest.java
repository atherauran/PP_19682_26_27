package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.RobotConfigs;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;

@TeleOp(name = "Servo Position Test", group = "Test")
public class ServoPositionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        String[] names = {RobotConfigs.HOOD, RobotConfigs.FEEDER};
        Servo[] servos = {
                hardwareMap.get(Servo.class, names[0]),
                hardwareMap.get(Servo.class, names[1])
        };
        double[] targets = {RobotConstants.HOOD_START, RobotConstants.FEEDER_REST};
        double[] lastCommands = {Double.NaN, Double.NaN};
        int selected = 0;

        telemetry.addLine("Disconnect servo linkages for the first test.");
        telemetry.addLine("No position commands until you press A after START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpadLeftWasPressed()) selected = 0;
            if (gamepad1.dpadRightWasPressed()) selected = 1;

            double step = gamepad1.left_bumper ? 0.001 : 0.01;
            if (gamepad1.dpadUpWasPressed()) targets[selected] += step;
            if (gamepad1.dpadDownWasPressed()) targets[selected] -= step;
            targets[selected] = Range.clip(targets[selected], 0, 1);

            if (gamepad1.aWasPressed()) {
                servos[selected].setPosition(targets[selected]);
                lastCommands[selected] = targets[selected];
            }

            telemetry.addData("Selected servo", names[selected]);
            telemetry.addData("Target position", "%.3f", targets[selected]);
            telemetry.addData("Step", "%.3f", step);
            for (int i = 0; i < servos.length; i++) {
                telemetry.addData(names[i] + " last command", Double.isNaN(lastCommands[i])
                        ? "Not commanded" : String.format(java.util.Locale.US, "%.3f", lastCommands[i]));
            }
            telemetry.addLine("D-pad left: hood | right: feeder");
            telemetry.addLine("D-pad up/down: adjust target | hold LB: fine steps");
            telemetry.addLine("A: move selected servo to target");
            telemetry.addLine("Full 0-1 range; watch for binding. Positions are commands, not measurements.");
            telemetry.addLine("Record values and copy them into RobotConstants.");
            telemetry.update();
            idle();
        }
    }
}
