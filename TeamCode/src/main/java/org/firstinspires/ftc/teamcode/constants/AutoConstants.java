package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class AutoConstants {
    private static final PoseFactory poses = PoseFactory.degrees();

    public static final Pose START_POSE = poses.of(24, 24, 0);
    public static final Pose FORWARD_POSE = poses.of(48, 24, 0);
    public static final Pose TURN_POSE = poses.of(48, 48, 90);
    public static final Pose CURVE_CONTROL = poses.of(48, 72, 0);
    public static final Pose RETURN_POSE = poses.of(24, 48, 180);
    public static final Pose PARK_POSE = poses.of(24, 24, 270);
}
