package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp
public class BlueTeleOp extends TeleOpBase {
    public static double AUDIENCE_CELL_X = 64;
    public static double FAR_CELL_X = 80;
    public static double CELL_Y = 59.25;

    public static double RESET_X = 36;
    public static double RESET_Y = 9;
    public static double RESET_H_DEG = 90;

    @Override
    protected String getAllianceName() { return "BLUE"; }

    @Override
    protected Pose getCell(boolean farCell) {
        return new Pose(farCell ? FAR_CELL_X : AUDIENCE_CELL_X, CELL_Y);
    }

    @Override
    protected boolean farCellStartsUp() { return true; }

    @Override
    protected Pose getRelocalizePose() {
        return new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_H_DEG));
    }
}
