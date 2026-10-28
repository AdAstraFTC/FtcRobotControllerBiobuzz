/* Copyright (c) 2026 Coach Chris Lemoine - Team00000 Mentor Bot
 *
 * Team00000 is a reference and demonstration platform, not a library for direct use.
 * Study this OpMode and recreate it in your own team's module.
 */

package org.firstinspires.ftc.team00000.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team00000.Hardware;

import java.util.ArrayList;
import java.util.List;

/**
 * Follow a route of straight lines and turns, then hold, using the Pinpoint pose.
 *
 * <p>Edit {@link #buildPath()} to stack the route. {@code axial} and {@code lateral}
 * each add one straight line along the current path heading. {@code linear} adds
 * one straight line that uses both axes at once. {@code yaw} turns in place.
 * The next path starts where the previous one ended.</p>
 *
 * <p>Constructs {@link Hardware} only. Hardware already owns the drive motors
 * and the Pinpoint. Leave Pedro's {@code Follower} out of this OpMode.</p>
 *
 * <p>The route is robot-start-relative. Pinpoint +X is the direction the
 * robot faced when pose was reset, +Y is left, and heading increases
 * counter-clockwise. On a path, positive axial is forward along the path
 * heading, positive lateral is right, and positive yaw is clockwise.
 * Leave room for every segment.</p>
 *
 * <p>Gains are starting values. Tune them from Panels on the field.</p>
 */
@Configurable
@Autonomous(name = "Forward", group = "Autonomous")
@SuppressWarnings("unused") // Instantiated by the FTC SDK from the @Autonomous annotation.
public class Forward extends OpMode {

    /** Length of the default forward path, in inches. */
    public static double FORWARD_INCHES = 100.0;

    /** Cruise power while the target is farther than {@link #SLOW_DOWN_INCHES}. */
    public static double MAX_POWER = 0.40;

    /**
     * Inside this distance, translation power falls from {@link #MAX_POWER}
     * toward {@link #MIN_POWER}. Raise it if the robot overshoots.
     */
    public static double SLOW_DOWN_INCHES = 6.0;

    /** Translation power floor outside the position tolerance, so friction does not stall the approach. */
    public static double MIN_POWER = 0.12;

    /** Heading correction in power per radian. The command is clockwise-positive, matching {@link Hardware#driveFieldCentric}. */
    public static double YAW_GAIN = 1.5;

    /** Cap on heading power so a large heading error cannot steal the whole drive. */
    public static double MAX_YAW_POWER = 0.30;

    /** Heading power floor outside the heading tolerance. */
    public static double MIN_YAW_POWER = 0.10;

    /** Stop translating when the planar error is inside this radius, in inches. */
    public static double POSITION_TOLERANCE_INCHES = 1.0;

    /** Stop turning when the heading error is inside this many degrees. */
    public static double HEADING_TOLERANCE_DEGREES = 3.0;

    private Hardware hardware;

    private int segmentIndex;
    private double axial;
    private double lateral;
    private double yaw;
    private boolean holding;
    private boolean calibrating;

    /**
     * Stack the autonomous here. Each call adds one path, in order.
     *
     * <p>{@code axial(24).yaw(90).axial(24)} drives forward 24 in, turns 90 degrees
     * clockwise, then drives forward 24 in along the new heading.
     * {@code linear(axial, lateral)} is one straight line using both axes.
     * A yaw takes the short way, so keep each yaw inside 180 degrees and chain
     * yaw paths for a larger turn.</p>
     *
     * <p>This runs every loop. Read constants here, not the live pose, so a
     * Panels edit moves the targets without restarting the route.</p>
     */
    private PathBuilder buildPath() {
        return new PathBuilder()
                .axial(FORWARD_INCHES);
    }

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
    }

    @Override
    public void init_loop() {
        hardware.updatePose();
        hardware.addInitTelemetry(telemetry);
        addRouteTelemetry(buildPath().resolve());
        telemetry.addLine("Press START to follow the route and hold.");
        telemetry.update();
    }

    @Override
    public void loop() {
        hardware.updatePose();
        List<Target> targets = buildPath().resolve();

        if (!hardware.isPinpointReady()) {
            hardware.stopDrive();
            axial = 0.0;
            lateral = 0.0;
            yaw = 0.0;
            holding = false;
            calibrating = true;
            telemetry.addLine("Pinpoint is still calibrating. Holding still.");
            addTelemetry(targets);
            telemetry.update();
            return;
        }
        calibrating = false;

        double x = hardware.getX(DistanceUnit.INCH);
        double y = hardware.getY(DistanceUnit.INCH);
        double heading = hardware.getHeading(AngleUnit.RADIANS);
        Target target = selectTarget(targets, x, y, heading);

        axial = 0.0;
        lateral = 0.0;
        yaw = 0.0;

        if (target == null) {
            holding = true;
            hardware.stopDrive();
            addTelemetry(targets);
            telemetry.update();
            return;
        }

        // Pinpoint field frame: +X forward at reset, +Y left, heading CCW.
        double forwardError = target.xInches - x;
        double leftError = target.yInches - y;
        double headingError = AngleUnit.normalizeRadians(target.headingRadians - heading);
        double distance = Math.hypot(forwardError, leftError);
        boolean onTarget = distance <= POSITION_TOLERANCE_INCHES
                && Math.abs(Math.toDegrees(headingError)) <= HEADING_TOLERANCE_DEGREES;
        holding = segmentIndex == targets.size() - 1 && onTarget;

        if (distance > POSITION_TOLERANCE_INCHES) {
            double slowDown = Math.max(SLOW_DOWN_INCHES, 0.1);
            double power = MAX_POWER * Math.min(1.0, distance / slowDown);
            power = Range.clip(Math.max(power, MIN_POWER), 0.0, 1.0);
            axial = power * forwardError / distance;
            // leftError is Pinpoint +Y. driveFieldCentric lateral is right-positive.
            lateral = power * (-leftError) / distance;
        }

        if (Math.abs(Math.toDegrees(headingError)) > HEADING_TOLERANCE_DEGREES) {
            // headingError is CCW-positive. driveFieldCentric yaw is clockwise-positive.
            double yawCommand = -headingError * YAW_GAIN;
            double yawCap = Range.clip(Math.abs(MAX_YAW_POWER), 0.0, 1.0);
            yaw = Range.clip(yawCommand, -yawCap, yawCap);
            if (Math.abs(yaw) < MIN_YAW_POWER) {
                yaw = Math.copySign(MIN_YAW_POWER, yawCommand);
            }
        }

        if (holding) {
            hardware.stopDrive();
        } else {
            hardware.driveFieldCentric(axial, lateral, yaw);
        }

        addTelemetry(targets);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (hardware != null) {
            hardware.stopDrive();
        }
    }

    private Target selectTarget(List<Target> targets, double xInches, double yInches, double headingRadians) {
        if (targets.isEmpty()) {
            segmentIndex = 0;
            return null;
        }
        if (segmentIndex < 0 || segmentIndex >= targets.size()) {
            segmentIndex = Math.max(0, Math.min(segmentIndex, targets.size() - 1));
        }
        while (segmentIndex < targets.size() - 1
                && withinTolerance(targets.get(segmentIndex), xInches, yInches, headingRadians)) {
            segmentIndex++;
        }
        return targets.get(segmentIndex);
    }

    private boolean withinTolerance(Target target, double xInches, double yInches, double headingRadians) {
        double distance = Math.hypot(target.xInches - xInches, target.yInches - yInches);
        double headingError = AngleUnit.normalizeRadians(target.headingRadians - headingRadians);
        return distance <= POSITION_TOLERANCE_INCHES
                && Math.abs(Math.toDegrees(headingError)) <= HEADING_TOLERANCE_DEGREES;
    }

    private void addRouteTelemetry(List<Target> targets) {
        if (targets.isEmpty()) {
            telemetry.addLine("Route is empty. The robot will hold.");
            return;
        }
        telemetry.addLine("Route from the start pose:");
        for (int i = 0; i < targets.size(); i++) {
            Target target = targets.get(i);
            telemetry.addData("Path " + (i + 1), "%s  ->  X %.1f  Y %.1f  H %.1f deg",
                    target.description,
                    target.xInches,
                    target.yInches,
                    Math.toDegrees(target.headingRadians));
        }
    }

    private void addTelemetry(List<Target> targets) {
        if (targets.isEmpty()) {
            telemetry.addData("Segment", "none");
        } else {
            int index = Math.max(0, Math.min(segmentIndex, targets.size() - 1));
            Target target = targets.get(index);
            telemetry.addData("Segment", "%d/%d %s", index + 1, targets.size(), target.description);
            telemetry.addData("Target", "X %.1f Y %.1f H %.1f deg",
                    target.xInches,
                    target.yInches,
                    Math.toDegrees(target.headingRadians));
        }
        String state = calibrating ? "CALIBRATING" : holding ? "HOLDING" : "DRIVING";
        telemetry.addData("State", state);
        telemetry.addData("Pose (in)", "X %.2f Y %.2f H %.1f deg",
                hardware.getX(DistanceUnit.INCH),
                hardware.getY(DistanceUnit.INCH),
                hardware.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Command", "axial %.2f lateral %.2f yaw %.2f", axial, lateral, yaw);
        hardware.addDriveTelemetry(telemetry);
    }

    /**
     * One straight line or one turn, chained from the end of the previous path.
     * Distances are inches. Yaw is degrees, clockwise-positive.
     */
    private static final class PathBuilder {
        private final List<Segment> segments = new ArrayList<>();

        /** Straight line along the path heading. Positive is forward. */
        PathBuilder axial(double inches) {
            return linear(inches, 0.0);
        }

        /** Straight line along the path heading. Positive is right. */
        PathBuilder lateral(double inches) {
            return linear(0.0, inches);
        }

        /**
         * One straight line in the path heading. Axial and lateral run together,
         * so the robot strafes along the diagonal. Positive axial is forward.
         * Positive lateral is right.
         */
        PathBuilder linear(double axialInches, double lateralInches) {
            segments.add(Segment.linear(axialInches, lateralInches));
            return this;
        }

        /**
         * Turn in place. Positive is clockwise. The turn takes the short way,
         * so keep this inside 180 degrees.
         */
        PathBuilder yaw(double clockwiseDegrees) {
            segments.add(Segment.yaw(clockwiseDegrees));
            return this;
        }

        List<Target> resolve() {
            double xInches = 0.0;
            double yInches = 0.0;
            double headingRadians = 0.0;
            List<Target> targets = new ArrayList<>();
            for (Segment segment : segments) {
                if (segment.kind == Segment.Kind.LINEAR) {
                    // Path heading is Pinpoint heading: 0 faces +X, positive is CCW.
                    // Axial follows that heading. Lateral is to the right, clockwise
                    // from forward, so its field vector is (sin(heading), -cos(heading)).
                    double cosine = Math.cos(headingRadians);
                    double sine = Math.sin(headingRadians);
                    xInches += segment.axialInches * cosine + segment.lateralInches * sine;
                    yInches += segment.axialInches * sine - segment.lateralInches * cosine;
                } else {
                    // Drive yaw is clockwise-positive. Pinpoint heading is CCW-positive.
                    headingRadians = AngleUnit.normalizeRadians(
                            headingRadians - Math.toRadians(segment.yawDegrees));
                }
                targets.add(new Target(
                        segment.description(), xInches, yInches, headingRadians));
            }
            return targets;
        }
    }

    private static final class Segment {
        enum Kind { LINEAR, YAW }

        final Kind kind;
        final double axialInches;
        final double lateralInches;
        final double yawDegrees;

        static Segment linear(double axialInches, double lateralInches) {
            return new Segment(Kind.LINEAR, axialInches, lateralInches, 0.0);
        }

        static Segment yaw(double clockwiseDegrees) {
            return new Segment(Kind.YAW, 0.0, 0.0, clockwiseDegrees);
        }

        private Segment(Kind kind, double axialInches, double lateralInches, double yawDegrees) {
            this.kind = kind;
            this.axialInches = axialInches;
            this.lateralInches = lateralInches;
            this.yawDegrees = yawDegrees;
        }

        String description() {
            if (kind == Kind.YAW) {
                String direction = yawDegrees >= 0.0 ? "CW" : "CCW";
                return String.format("yaw %.1f deg %s", Math.abs(yawDegrees), direction);
            }
            if (lateralInches == 0.0) {
                return String.format("axial %.1f in", axialInches);
            }
            if (axialInches == 0.0) {
                return String.format("lateral %.1f in", lateralInches);
            }
            return String.format("linear axial %.1f in, lateral %.1f in", axialInches, lateralInches);
        }
    }

    private static final class Target {
        final String description;
        final double xInches;
        final double yInches;
        final double headingRadians;

        Target(String description, double xInches, double yInches, double headingRadians) {
            this.description = description;
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingRadians = headingRadians;
        }
    }
}
