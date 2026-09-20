/* Copyright (c) 2026 Coach Chris Lemoine - Team00000 Mentor Bot
 *
 * Team00000 is a reference and demonstration platform, not a library for direct use.
 *
 * Purpose:
 *  - Coaches and mentors use this robot and codebase to test ideas, validate patterns,
 *    and demonstrate best practices.
 *  - Students are expected to study this code and recreate equivalent functionality
 *    in their own team's codebase (team31192, team36103, team36104, etc.).
 *
 * We extract strong patterns from the official samples and implement them cleanly here
 * as a teaching reference rather than editing the external samples directly.
 */

package org.firstinspires.ftc.team00000;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Hardware - Primary hardware abstraction for the mentor bot.
 *
 * <p>Provides a clean, reusable interface for controlling four mecanum drive motors
 * and reading pose data from the goBILDA Pinpoint odometry computer.</p>
 *
 * <p>{@link Config} is the single source of truth for drivetrain and Pinpoint wiring.
 * Teleop reads it here; Pedro Pathing autonomous reads the same fields from
 * {@code Constants.createFollower()}.</p>
 *
 * <p>Students and OpModes should interact with this class through its public methods
 * rather than accessing motors or the Pinpoint directly. {@link #getPinpoint()} is an
 * advanced escape hatch only.</p>
 */
public class Hardware {

    /**
     * Robot wiring and Pinpoint geometry. Edit these values — not copies in Pedro
     * {@code Constants}. Signs follow the official goBILDA Pinpoint sample and the
     * Pedro Pathing Pinpoint localizer (externally measured).
     */
    @Configurable
    public static class Config {
        public static String FRONT_LEFT_NAME = "frontLeftDrive";
        public static String FRONT_RIGHT_NAME = "frontRightDrive";
        public static String BACK_LEFT_NAME = "backLeftDrive";
        public static String BACK_RIGHT_NAME = "backRightDrive";
        public static String PINPOINT_NAME = "pinpoint";

        public static DcMotor.Direction FRONT_LEFT_DIRECTION = DcMotor.Direction.REVERSE;
        public static DcMotor.Direction FRONT_RIGHT_DIRECTION = DcMotor.Direction.FORWARD;
        public static DcMotor.Direction BACK_LEFT_DIRECTION = DcMotor.Direction.REVERSE;
        public static DcMotor.Direction BACK_RIGHT_DIRECTION = DcMotor.Direction.FORWARD;

        public static double DRIVE_MAX_POWER = 1.0;

        /*
         * PINPOINT POD OFFSETS (millimeters)
         *
         * These are the locations of the two odometry pods relative to the tracking
         * point (center of rotation), not the location of the Pinpoint computer itself.
         *
         * Official goBILDA setOffsets(xOffset, yOffset):
         *   xOffset = how far sideways the FORWARD (X) pod is. Left +, right -.
         *   yOffset = how far forward the STRAFE (Y) pod is. Forward +, back -.
         *
         * Pedro Pathing uses the same numbers under different names:
         *   forwardPodY == xOffset (Y is left in robot coordinates)
         *   strafePodX  == yOffset (X is forward in robot coordinates)
         *
         * Values below are the externally measured Pedro Pathing pair.
         */
        public static double FORWARD_POD_Y_MM = -153.50;
        public static double STRAFE_POD_X_MM = 56.00;
        public static DistanceUnit PINPOINT_DISTANCE_UNIT = DistanceUnit.MM;
        public static GoBildaOdometryPods PODS = GoBildaOdometryPods.goBILDA_4_BAR_POD;

        /*
         * ENCODER DIRECTIONS
         *
         * The forward (X) pod should increase when the robot moves forward.
         * The strafe (Y) pod should increase when the robot moves left.
         * Incorrect directions invert heading or strafe.
         */
        public static EncoderDirection FORWARD_ENCODER_DIRECTION = EncoderDirection.FORWARD;
        public static EncoderDirection STRAFE_ENCODER_DIRECTION = EncoderDirection.REVERSED;
    }

    /* =====================================================
     * HARDWARE OBJECTS
     * ===================================================== */
    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor backRightDrive;
    private final GoBildaPinpointDriver pinpoint;

    /* =====================================================
     * CONSTRUCTOR
     * ===================================================== */
    public Hardware(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, Config.FRONT_LEFT_NAME);
        frontRightDrive = hardwareMap.get(DcMotor.class, Config.FRONT_RIGHT_NAME);
        backLeftDrive = hardwareMap.get(DcMotor.class, Config.BACK_LEFT_NAME);
        backRightDrive = hardwareMap.get(DcMotor.class, Config.BACK_RIGHT_NAME);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Config.PINPOINT_NAME);

        initDriveMotors();
        initPinpoint();
    }

    /* =====================================================
     * INITIALIZATION
     * ===================================================== */

    private void initDriveMotors() {
        /*
         * MOTOR DIRECTIONS FOR STANDARD MECANUM (X-drive pattern)
         *
         * Viewed from above, the wheels should form an "X" roller pattern.
         * These directions assume direct drive (no extra gearing that reverses rotation).
         *
         * TEST PROCEDURE:
         *   Push the left stick forward
         *   - If the robot drives backward, flip the direction of ALL four motors.
         *   - If strafing is wrong, adjust the left vs right pairs.
         */
        frontLeftDrive.setDirection(Config.FRONT_LEFT_DIRECTION);
        frontRightDrive.setDirection(Config.FRONT_RIGHT_DIRECTION);
        backLeftDrive.setDirection(Config.BACK_LEFT_DIRECTION);
        backRightDrive.setDirection(Config.BACK_RIGHT_DIRECTION);

        // Brake is more predictable than coast when using odometry for positioning.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // We rely on the Pinpoint for position, not the motor encoders.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initPinpoint() {
        // xOffset = forward pod Y (left +); yOffset = strafe pod X (forward +).
        pinpoint.setOffsets(Config.FORWARD_POD_Y_MM, Config.STRAFE_POD_X_MM, Config.PINPOINT_DISTANCE_UNIT);
        pinpoint.setEncoderResolution(Config.PODS);
        pinpoint.setEncoderDirections(Config.FORWARD_ENCODER_DIRECTION, Config.STRAFE_ENCODER_DIRECTION);

        /*
         * Reset position and IMU once at construction. Do not sleep here.
         * Poll isPinpointReady() from the OpMode init_loop() and wait until
         * the Driver Station shows READY before pressing START.
         */
        pinpoint.resetPosAndIMU();
    }

    /* =====================================================
     * PUBLIC DRIVE API
     * ===================================================== */

    /**
     * Drives the robot in a robot-centric manner.
     * Movement is relative to the robot's current orientation.
     *
     * @param axial   Forward (+) / backward (-) power [-1.0, 1.0]
     * @param lateral Right (+) / left (-) strafe power [-1.0, 1.0]
     *                (same sign as {@code gamepad.left_stick_x})
     * @param yaw     Clockwise (+) / counter-clockwise (-) rotation power [-1.0, 1.0]
     *                (same sign as {@code gamepad.right_stick_x})
     */
    public void driveRobotCentric(double axial, double lateral, double yaw) {
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        /*
         * Normalize wheel powers so no individual wheel exceeds ±1.0.
         * This preserves the intended direction of travel even when
         * the combined request would otherwise saturate one or more motors.
         */
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        setDrivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /**
     * Drives the robot field-relatively using the current Pinpoint heading.
     *
     * <p>"Forward" on the joystick is the heading-zero direction — the way the
     * robot faced when pose was last reset — not automatically FTC field +Y.
     * To drive in true field coordinates, call {@link #getPinpoint()} and
     * {@code setPosition} to a field starting pose before enabling this mode.</p>
     *
     * @param axial   Forward (+) / backward (-) in the heading-zero frame [-1.0, 1.0]
     * @param lateral Right (+) / left (-) strafe in the heading-zero frame [-1.0, 1.0]
     *                (same sign as {@code gamepad.left_stick_x})
     * @param yaw     Clockwise (+) / counter-clockwise (-) rotation [-1.0, 1.0]
     *                (same sign as {@code gamepad.right_stick_x})
     */
    public void driveFieldCentric(double axial, double lateral, double yaw) {
        double theta = Math.atan2(axial, lateral);
        double r = Math.hypot(lateral, axial);
        theta = AngleUnit.normalizeRadians(theta - getHeading(AngleUnit.RADIANS));

        // Rotate translational inputs by the inverse of the robot's current heading
        double rotatedAxial   = r * Math.sin(theta);
        double rotatedLateral = r * Math.cos(theta);

        driveRobotCentric(rotatedAxial, rotatedLateral, yaw);
    }

    /**
     * Directly sets power to each of the four drive motors.
     * Useful for debugging or when fine-grained control is needed.
     */
    public void setDrivePower(double frontLeft, double frontRight, double backLeft, double backRight) {
        double cap = Math.min(1.0, Math.abs(Config.DRIVE_MAX_POWER));
        frontLeftDrive.setPower(Range.clip(frontLeft, -cap, cap));
        frontRightDrive.setPower(Range.clip(frontRight, -cap, cap));
        backLeftDrive.setPower(Range.clip(backLeft, -cap, cap));
        backRightDrive.setPower(Range.clip(backRight, -cap, cap));
    }

    public void stopDrive() {
        setDrivePower(0.0, 0.0, 0.0, 0.0);
    }

    /* =====================================================
     * PUBLIC ODOMETRY / PINPOINT API
     * ===================================================== */

    /**
     * Updates the Pinpoint's internal pose calculations from the pods and IMU.
     * Must be called every loop iteration before reading pose data.
     */
    public void updatePose() {
        pinpoint.update();
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }

    public double getX(DistanceUnit unit) {
        return getPose().getX(unit);
    }

    public double getY(DistanceUnit unit) {
        return getPose().getY(unit);
    }

    public double getHeading(AngleUnit unit) {
        return getPose().getHeading(unit);
    }

    /**
     * Resets the Pinpoint position and IMU heading to zero and starts IMU
     * recalibration. The robot must be stationary. Poll {@link #isPinpointReady()}
     * from {@code init_loop()} before using heading for field-centric drive.
     */
    public void resetPose() {
        pinpoint.resetPosAndIMU();
    }

    /**
     * Returns true if the Pinpoint has completed IMU calibration and is ready
     * to provide high-accuracy pose data.
     */
    public boolean isPinpointReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    /**
     * Same as {@link #resetPose()}. Named alias for an IMU recalibration.
     */
    public void recalibratePinpoint() {
        resetPose();
    }

    /**
     * Advanced escape hatch for the raw Pinpoint driver (for example
     * {@code setPosition} to a known field pose). Prefer the methods above.
     */
    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    /* =====================================================
     * TELEMETRY HELPERS
     * ===================================================== */

    public void addDriveTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive", "FL %.2f FR %.2f BL %.2f BR %.2f",
                frontLeftDrive.getPower(),
                frontRightDrive.getPower(),
                backLeftDrive.getPower(),
                backRightDrive.getPower());
    }

    public void addPoseTelemetry(Telemetry telemetry, DistanceUnit distUnit, AngleUnit angleUnit) {
        Pose2D pose = getPose();
        telemetry.addData("Pose", "X %.2f Y %.2f H %.1f",
                pose.getX(distUnit),
                pose.getY(distUnit),
                pose.getHeading(angleUnit));
    }

    /**
     * Pinpoint calibration status for {@code init_loop()}. Wait until READY
     * before pressing START if field-centric heading is required.
     */
    public void addInitTelemetry(Telemetry telemetry) {
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        telemetry.addData("Pinpoint", isPinpointReady() ? "READY - press START" : status.toString());
        addPoseTelemetry(telemetry, DistanceUnit.MM, AngleUnit.DEGREES);
    }
}
