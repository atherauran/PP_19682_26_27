package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.RobotConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.controllers.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class AutoAim {

    public static class TurretCommand {
        public boolean hasTarget = false;
        public double targetDist = 0.0;
        public double targetTurretAngle = 0.0;
        public double aimError = 0.0;

        void reset() {
            hasTarget = false;
            targetDist = 0.0;
            targetTurretAngle = 0.0;
            aimError = 0.0;
        }
    }

    public static class EmaFilter {
        private double value;
        private boolean init;
        private final double alpha;

        public EmaFilter(double alpha) {
            this.alpha = alpha;
        }

        public double update(double raw) {
            if (!init) { value = raw; init = true; }
            else        value += alpha * (raw - value);
            return value;
        }

        public double get() { return value; }
        public void reset() { init = false; value = 0.0; }
        public boolean isInit() { return init; }
    }

    @Configurable
    public static class TurretController {
        public static double kP = 10.0, kI = 0.0, kD = 0.5;
        public static double kV = 0.001217, kS = 0.057978, kA = 0.000006;
        public static double kLinearBraking = 0.046419;
        public static double MAX_POWER = 1.0;
        public static double TUNING_VOLTAGE = 13.84;
        public static double ANGLE_ALPHA = 0.8, VEL_ALPHA = 0.8, ACCEL_ALPHA = 0.15;

        private Turret turret;
        private HardwareMap hwMap;

        private final PIDController pid = new PIDController(kP, kI, kD);
        private final EmaFilter angleF = new EmaFilter(ANGLE_ALPHA);
        private final EmaFilter velF   = new EmaFilter(VEL_ALPHA);
        private final EmaFilter accelF = new EmaFilter(ACCEL_ALPHA);

        private double lastRawAngle, lastTargetVel;
        private long lastVoltTime;
        private double battery = 12.0;

        public void init(Turret turret, HardwareMap hm) {
            this.turret = turret;
            hwMap = hm;
            battery = readBattery();
        }

        public void update(double curAngle, double tgtAngle, double ffVel, double dt) {
            dt = clampDt(dt);
            pid.kP = kP; pid.kI = kI; pid.kD = kD;
            refVolt();
            updateFilters(dt);

            tgtAngle = Range.clip(tgtAngle, minAngle(), maxAngle());

            double pidVel   = pid.calculate(tgtAngle, tgtAngle - curAngle);
            double tgtVel   = pidVel + ffVel;
            double rawAccel = (tgtVel - lastTargetVel) / dt;
            lastTargetVel = tgtVel;
            double filtAccel = accelF.update(rawAccel);

            double ks = Math.abs(tgtVel) > 0.5 ? Math.signum(tgtVel) : 0.0;
            double pwr = filtAccel * kA + tgtVel * kV + ks * kS;
            pwr *= TUNING_VOLTAGE / battery;
            turret.setPower(Range.clip(pwr, -MAX_POWER, MAX_POWER));
        }

        public double predictBraking(double curAngle, double error) {
            double v = velF.get();
            double d = kLinearBraking * Math.abs(v);
            if (Math.signum(error) != Math.signum(v) && Math.abs(v) > 0.5) {
                return curAngle + Math.signum(v) * d;
            }
            return curAngle;
        }

        public void halt() { turret.setPower(0.0); }
        public double rawAngle() { return turret.getDegrees(); }
        public double filtAngle() { return angleF.get(); }
        public double filtVel()   { return velF.get(); }
        public double getBattery(){ return battery; }

        public void reset() {
            angleF.reset(); velF.reset(); accelF.reset();
            pid.reset();
            lastTargetVel = 0;
        }

        private void updateFilters(double dt) {
            double raw = rawAngle();
            if (angleF.isInit()) velF.update((raw - lastRawAngle) / dt);
            angleF.update(raw);
            lastRawAngle = raw;
        }

        private void refVolt() {
            long n = System.nanoTime();
            if (n - lastVoltTime > 250_000_000L) { battery = readBattery(); lastVoltTime = n; }
        }

        private double readBattery() {
            double m = 0;
            for (VoltageSensor s : hwMap.voltageSensor) m = Math.max(m, s.getVoltage());
            return Math.max(8, m > 0 ? m : TUNING_VOLTAGE);
        }

        private static double clampDt(double dt) {
            return Range.clip(dt, 0.005, 0.100);
        }
    }

    private final TurretController turret = new TurretController();
    private final TurretCommand cmd = new TurretCommand();
    private long lastTime = 0;

    public void init(Turret turretSubsystem, HardwareMap hm) {
        turret.init(turretSubsystem, hm);
    }

    public TurretCommand update(double turretX, double turretY, double headingDeg, double omegaDeg,
                                double targetX, double targetY, double yawOffset) {
        cmd.reset();

        long now = System.nanoTime();
        double dt = lastTime == 0 ? 0.0001 : (now - lastTime) / 1e9;
        lastTime = now;

        double dx = targetX - turretX, dy = targetY - turretY;
        double dist = Math.hypot(dx, dy);
        if (dist < 0.1) {
            turret.halt();
            return cmd;
        }
        cmd.hasTarget = true;
        cmd.targetDist = dist;

        double absTarget = Math.toDegrees(Math.atan2(dy, dx)) + yawOffset;
        double error = normalizeDeg(absTarget - headingDeg - turret.filtAngle());
        double relTarget = turret.filtAngle() + error;
        if (!inRange(relTarget)) {
            relTarget -= Math.copySign(360.0, error);
            error = relTarget - turret.filtAngle();
        }

        cmd.targetTurretAngle = relTarget;
        cmd.aimError = error;

        if (!inRange(relTarget)) {
            cmd.hasTarget = false;
            cmd.targetTurretAngle = 0.0;
            turret.update(turret.filtAngle(), 0.0, 0.0, dt);
            return cmd;
        }

        double predictedAngle = turret.predictBraking(turret.filtAngle(), error);
        turret.update(predictedAngle, relTarget, -omegaDeg, dt);
        return cmd;
    }

    public double getCurrentTurretAngle()  { return turret.rawAngle(); }
    public double getFilteredTurretAngle() { return turret.filtAngle(); }
    public double getFilteredTurretVelocity() { return turret.filtVel(); }
    public double getCurrentBatteryVoltage()  { return turret.getBattery(); }

    public void reset() {
        turret.reset();
        lastTime = 0;
    }

    private static boolean inRange(double angle) {
        return angle >= minAngle() && angle <= maxAngle();
    }

    private static double minAngle() { return Turret.toDegrees(TURRET_MIN_TICKS); }
    private static double maxAngle() { return Turret.toDegrees(TURRET_MAX_TICKS); }

    private static double normalizeDeg(double a) {
        while (a > 180.0) a -= 360.0;
        while (a <= -180.0) a += 360.0;
        return a;
    }
}
