package org.firstinspires.ftc.team36104;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Hardware {

    /* =====================================================
     * HARDWARE OBJECTS
     * ===================================================== */
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private IMU imu = null;

    /* =====================================================
     * CONFIGURATION CONSTANTS
     *
     * These values are centralized here so they only need to be changed in one place.
     * Motor names must match the Driver Station configuration exactly.
     * ===================================================== */
    public static final String FRONT_LEFT_NAME = "frontLeftDrive";
    public static final String FRONT_RIGHT_NAME = "frontRightDrive";
    public static final String BACK_LEFT_NAME = "backLeftDrive";
    public static final String BACK_RIGHT_NAME = "backRightDrive";
    public static final String IMU_NAME = "imu";

    /*
     * REV HUB ORIENTATION ON THE ROBOT
     *
     * These describe how the Control Hub (or Expansion Hub with the IMU) is mounted.
     * They MUST match the physical robot or field-centric driving and heading turns
     * will be wrong.
     *
     * Common Control Hub mounting:
     *   Logo facing FORWARD (toward the front of the robot)
     *   USB ports facing UP
     *
     * Use ConceptExploringIMUOrientation or SensorIMUOrthogonal to verify.
     */
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    /* =====================================================
     * CONSTRUCTOR
     * ===================================================== */
    public Hardware(HardwareMap hardwareMap) {
        initDriveMotors(hardwareMap);
        initImu(hardwareMap);
    }

    /* =====================================================
     * INITIALIZATION
     * ===================================================== */

    private void initDriveMotors (HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRightDrive = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeftDrive = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRightDrive = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        /*
         * MOTOR DIRECTIONS FOR STANDARD MECANUM (X-drive pattern)
         *
         * Viewed from above, the wheels should form an "X" roller pattern.
         * These directions assume direct drive (no extra gearing that reverses rotation).
         *
         * TEST PROCEDURE:
         *   Push the left stick forward
         *   - If the robot drives backward, flip the direction od ALL four motors.
         *   - If strafing is wrong, adjust the left vs right pairs.
         */
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Brake is more predictable than coast when holding heading or stopping for turns.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TeleOp drive uses open-loop power; heading comes from the IMU, not motor encoders.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initImu (HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, IMU_NAME);

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(LOGO_FACING, USB_FACING);
        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        /*
         * Initialize the IMU with the hub's mounting orientation.
         *
         * initialize() returns quickly; the IMU is typically ready immediately
         * for heading reads. Prefer resetYaw() at the start of a match (or on a
         * gamepad button) rather than blocking sleeps in this hardware class.
         */
        imu.initialize(parameters);
        imu.resetYaw();
    }

    /* =====================================================
     * PUBLIC DRIVE API
     * ===================================================== */

    /**
     * Drives the robot in a **robot-centric** manner.
     * Movement is relative to the robot's current orientation.
     *
     * @param axial   Forward (+) / backward (-) power [-1.0, 1.0]
     * @param lateral Left (+) / right (-) strafe power [-1.0, 1.0]
     * @param yaw     Clockwise (+) / counter-clockwise (-) rotation power [-1.0, 1.0]
     */
    public void driveRobotCentric(double axial, double lateral, double yaw) {
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        /*
         * Normalize wheel powers so no individual wheel exceeds ±1.0.
         * This preserves the intended direction of travel even when
         * the combined request would otherwise saturate on or more motors.
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
     * Drives the robot in a **field-centric** manner.
     * "Forward" on the joystick always moves the robot toward the positive Y-axis on the field,
     * regardless of the robot's current rotation.
     *
     * @param axial   Forward (+) / backward (-) power relative to the field [-1.0, 1.0]
     * @param lateral Left (+) / right (-) strafe power relative to the field [-1.0, 1.0]
     * @param yaw     Clockwise (+) / counter-clockwise (-) rotation power [-1.0, 1.0]
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
        frontLeftDrive.setPower(Range.clip(frontLeft, -1.0, 1.0));
        frontRightDrive.setPower(Range.clip(frontRight, -1.0, 1.0));
        backLeftDrive.setPower(Range.clip(backLeft, -1.0, 1.0));
        backRightDrive.setPower(Range.clip(backRight, -1.0, 1.0));
    }

    public void stopDrive() {
        setDrivePower(0.0, 0.0, 0.0, 0.0);
    }

    /* =====================================================
     * PUBLIC IMU / HEADING API
     * ===================================================== */

    /**
     * No-op kept for OpMode compatibility with the former Pinpoint-based API.
     * The REV IMU does not require a periodic update() call before reading heading.
     */
    public void updateHeading() {
        // Intentionally empty — IMU heading is read on demand.
    }

    /**
     * @deprecated Prefer {@link #updateHeading()}. Kept so existing OpModes that called
     * updatePose() still compile while migrating off Pinpoint.
     */
    @Deprecated
    public void updatePose() {
        updateHeading();
    }

    public YawPitchRollAngles getOrientation() {
        if (imu != null) {
            return imu.getRobotYawPitchRollAngles();
        }
        return new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
    }

    public double getHeading(AngleUnit unit) {
        return getOrientation().getYaw(unit);
    }

    public double getPitch(AngleUnit unit) {
        return getOrientation().getPitch(unit);
    }

    public double getRoll(AngleUnit unit) {
        return getOrientation().getRoll(unit);
    }

    /**
     * Resets the IMU yaw (heading) to zero.
     * Call this at the start of autonomous / TeleOp, or when the driver wants to
     * re-zero field-centric "forward" (commonly bound to a gamepad button).
     */
    public void resetHeading() {
        if (imu != null) {
            imu.resetYaw();
        }
    }

    /**
     * @deprecated Prefer {@link #resetHeading()}.
     */
    @Deprecated
    public void resetPose() {
        resetHeading();
    }

    /**
     * Returns true if the IMU object was obtained from the hardware map.
     * Unlike the Pinpoint, the universal IMU API does not expose a DeviceStatus enum;
     * treat a successful initialize() + non-null handle as ready for heading reads.
     */
    public boolean isImuReady() {
        return imu != null;
    }

    /**
     * @deprecated Prefer {@link #isImuReady()}.
     */
    @Deprecated
    public boolean isPinpointReady() {
        return isImuReady();
    }

    /**
     * Re-zeros yaw. Useful after the robot is disturbed or when re-establishing
     * field-centric forward mid-match.
     */
    public void recalibrateImu() {
        resetHeading();
    }

    /**
     * @deprecated Prefer {@link #recalibrateImu()}.
     */
    @Deprecated
    public void recalibratePinpoint() {
        recalibrateImu();
    }

    /**
     * Returns the raw IMU for advanced use cases.
     * Most students should use the higher-level methods above instead.
     */
    public IMU getImu() {
        return imu;
    }

    /* =====================================================
     * TELEMETRY HELPERS
     * ===================================================== */

    public void addDriveTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Drive", "FL %.2f FR %.2f BL %.2f BR %.2f",
                frontLeftDrive.getPower(),
                frontRightDrive.getPower(),
                backLeftDrive.getPower(),
                backRightDrive.getPower());
    }

    public void addHeadingTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry,
                                    AngleUnit angleUnit) {
        YawPitchRollAngles orientation = getOrientation();
        telemetry.addData("IMU", "Y %.1f  P %.1f  R %.1f",
                orientation.getYaw(angleUnit),
                orientation.getPitch(angleUnit),
                orientation.getRoll(angleUnit));
    }

    /**
     * @deprecated Prefer {@link #addHeadingTelemetry}. X/Y pose is not available from the REV IMU.
     */
    @Deprecated
    public void addPoseTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry,
                                 org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit distUnit,
                                 AngleUnit angleUnit) {
        // distUnit unused — IMU has no X/Y position
        addHeadingTelemetry(telemetry, angleUnit);
    }
}
