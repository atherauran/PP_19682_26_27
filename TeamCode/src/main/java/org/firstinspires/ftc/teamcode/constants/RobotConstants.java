package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Configurable
public class RobotConstants {
    public static Pose autoEndPose = Pose.zero();

    public static double DRIVE_POWER = 0.6;
    public static double SLOW_DRIVE_POWER = 0.25;
    public static double STICK_DEADBAND = 0.05;
    public static double INTAKE_POWER = 0.7;
    public static double FEED_INTAKE_POWER = 0.5;

    // Motor encoder ticks/second, not RPM. These are initial bench settings.
    public static double SHOOTER_VELOCITY = 800;
    public static double SHOOTER_IDLE_VELOCITY = 200; // Set to 0 to disable idle spin.
    public static double SHOOTER_VELOCITY_STEP = 50;
    public static double SHOOTER_MAX_VELOCITY = 2000;
    public static double SHOOTER_VELOCITY_TOLERANCE = 75;

    // Optional PIDF starting point for approximately 2600 ticks/s maximum speed.
    // F = 32767 / measured max ticks/s, P = F / 10, I = P / 10, D = 0.
    public static double SHOOTER_KP = 1.26;
    public static double SHOOTER_KI = 0.126;
    public static double SHOOTER_KD = 0;
    public static double SHOOTER_KF = 12.6;

    // Provisional servo positions: calibrate with horns/linkages disconnected first.
    public static double HOOD_START = 0.5;
    public static double HOOD_MIN = 0.25;
    public static double HOOD_MAX = 0.75;
    public static double HOOD_STEP = 0.01;
    public static double FEEDER_REST = 0.5;
    public static double FEEDER_FEED = 0.6;
    public static double FEEDER_PULSE_MS = 250;

    public static double TURRET_MANUAL_POWER = 0.15;
    // Zero the encoder with the turret facing forward. Measure cable-safe travel in ticks.
    // With limits disabled, the driver must stop before cables tighten.
    public static boolean TURRET_LIMITS_ENABLED = false;
    public static int TURRET_MIN_TICKS = 0;
    public static int TURRET_MAX_TICKS = 0;
}
