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

/**
 * Drive forward a fixed distance and hold, using the Pinpoint pose.
 *
 * <p>Constructs {@link Hardware} only. Hardware already owns the drive motors
 * and the Pinpoint, so this OpMode must not also construct a Pedro
 * {@code Follower}.</p>
 *
 * <p>The target is robot-start-relative. Pinpoint +X is the direction the
 * robot faced when pose was reset, +Y is left, and heading increases
 * counter-clockwise. Heading 0 is held for the whole drive. Place the robot
 * with a clear path ahead.</p>
 *
 * <p>Gains are starting values. Tune them from Panels on the field.</p>
 */
@Configurable
@Autonomous(name = "Forward", group = "Autonomous")
@SuppressWarnings("unused") // Instantiated by the FTC SDK from the @Autonomous annotation.
public class Forward extends OpMode {

    /** Distance to drive along Pinpoint +X, in inches. */
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

    private double axial;
    private double lateral;
    private double yaw;
    private boolean holding;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
    }

    @Override
    public void init_loop() {
        hardware.updatePose();
        hardware.addInitTelemetry(telemetry);
        telemetry.addLine("Clear " + FORWARD_INCHES + " in ahead of the robot.");
        telemetry.addLine("Press START to drive forward and hold.");
        telemetry.update();
    }

    @Override
    public void loop() {
        hardware.updatePose();

        if (!hardware.isPinpointReady()) {
            hardware.stopDrive();
            axial = 0.0;
            lateral = 0.0;
            yaw = 0.0;
            holding = false;
            telemetry.addLine("Pinpoint is still calibrating. Holding still.");
            addTelemetry();
            telemetry.update();
            return;
        }

        double x = hardware.getX(DistanceUnit.INCH);
        double y = hardware.getY(DistanceUnit.INCH);
        double heading = hardware.getHeading(AngleUnit.RADIANS);

        // Pinpoint field frame: +X forward at reset, +Y left, heading CCW.
        double forwardError = FORWARD_INCHES - x;
        double leftError = -y;
        double headingError = AngleUnit.normalizeRadians(-heading);
        double distance = Math.hypot(forwardError, leftError);

        holding = distance <= POSITION_TOLERANCE_INCHES
                && Math.abs(Math.toDegrees(headingError)) <= HEADING_TOLERANCE_DEGREES;

        axial = 0.0;
        lateral = 0.0;
        yaw = 0.0;

        if (distance > POSITION_TOLERANCE_INCHES) {
            double slowDown = Math.max(SLOW_DOWN_INCHES, 0.1);
            double power = MAX_POWER * Math.min(1.0, distance / slowDown);
            power = Range.clip(Math.max(power, MIN_POWER), 0.0, 1.0);
            axial = power * forwardError / distance;
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

        addTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (hardware != null) {
            hardware.stopDrive();
        }
    }

    private void addTelemetry() {
        telemetry.addData("State", holding ? "HOLDING" : "DRIVING");
        telemetry.addData("Target (in)", FORWARD_INCHES);
        telemetry.addData("Pose (in)", "X %.2f Y %.2f H %.1f deg",
                hardware.getX(DistanceUnit.INCH),
                hardware.getY(DistanceUnit.INCH),
                hardware.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Command", "axial %.2f lateral %.2f yaw %.2f", axial, lateral, yaw);
        hardware.addDriveTelemetry(telemetry);
    }
}
