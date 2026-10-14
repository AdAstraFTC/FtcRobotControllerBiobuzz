/* Copyright (c) 2026 Coach Chris Lemoine - Team00000 Mentor Bot
 *
 * Team00000 is a reference and demonstration platform, not a library for direct use.
 *
 * Purpose:
 *  - Coaches and mentors use this robot and codebase to test ideas, validate patterns,
 *    and demonstrate best preactices.
 *  - Students are expected to study this code and recreate equivalent functionality
 *    in their own team's codebase (team31192, team36103, team36104, etc.).
 *
 * We extract strong patters from the official samples and implenet them cleanly here
 * as a teaching reference rather than editing the external.samples directly.
 */

package org.firstinspires.ftc.team31192;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Hardware {

    /* =====================================================
     * HARDWARE OBJECTS
     * ===================================================== */
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotor kickerMotor = null;
    private DcMotor flywheelMotor = null;
    private Servo launcherSpine = null;
    private GoBildaPinpointDriver pinpoint = null;

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
    public static final String INTAKE_NAME = "intakeMotor";
    public static final String KICKER_NAME = "kickerMotor";
    public static final String FLYWHEEL_NAME = "flywheelMotor";
    public static final String LAUNCHER_SPINE_NAME = "launcherSpine";
    public static final String PINPOINT_NAME = "pinpoint";

    public static final DcMotor.Direction INTAKE_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction KICKER_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction FLYWHEEL_DIRECTION = DcMotor.Direction.FORWARD;

    /** Held power while testing direction. Raise this after the spin direction is confirmed. */
    public static final double INTAKE_POWER = 1.0;
    public static final double KICKER_POWER = 1.0;
    public static final double FLYWHEEL_POWER = 0.5;
    public static final double SPINE_POLLEN_POSITION = 0.5;
    public static final double SPINE_NECTAR_POSITION = 0.0;

    public static final DcMotor.Direction FRONT_LEFT_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction FRONT_RIGHT_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction BACK_LEFT_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction BACK_RIGHT_DIRECTION = DcMotor.Direction.FORWARD;

    /*
     * PINPOINT POD OFFSETS (millimeters)
     *
     * These are the odometry pods relative to the center of rotation, not the
     * Pinpoint computer. Official goBILDA setOffsets(xOffset, yOffset):
     *   xOffset = how far sideways the FORWARD pod is. Left +, right -.
     *   yOffset = how far forward the STRAFE pod is. Forward +, back -.
     *
     * Pedro reads these same fields. Teleop calls setOffsets with them in this order.
     */
    public static final double FORWARD_POD_Y_MM = -153.50;
    public static final double STRAFE_POD_X_MM = 56.00;
    public static final GoBildaOdometryPods PODS = GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /*
     * ENCODER DIRECTIONS
     *
     * The forward pod should increase when the robot moves forward.
     * The strafe pod should increase when the robot moves left.
     */
    public static final EncoderDirection FORWARD_ENCODER_DIRECTION = EncoderDirection.FORWARD;
    public static final EncoderDirection STRAFE_ENCODER_DIRECTION = EncoderDirection.REVERSED;

    /* =====================================================
     * CONSTRUCTOR
     * ===================================================== */
    public Hardware(HardwareMap hardwareMap) {
        initDriveMotors(hardwareMap);
        initIntakeMotors(hardwareMap);
        initFlywheelMotors(hardwareMap);
        initLauncherSpine(hardwareMap);
        initPinpoint(hardwareMap);
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
        frontLeftDrive.setDirection(FRONT_LEFT_DIRECTION);
        frontRightDrive.setDirection(FRONT_RIGHT_DIRECTION);
        backLeftDrive.setDirection(BACK_LEFT_DIRECTION);
        backRightDrive.setDirection(BACK_RIGHT_DIRECTION);

        // Brake is more predictable than coast when using odometry for positioning.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // We rely on the Pinpoint for Position, not the motor encoders.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void initIntakeMotors (HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_NAME);
        kickerMotor = hardwareMap.get(DcMotor.class, KICKER_NAME);

        intakeMotor.setDirection(INTAKE_DIRECTION);
        kickerMotor.setDirection(KICKER_DIRECTION);

        // Coast if a piece jams the roller. Brake so the kicker stops when the trigger is released.
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        kickerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void initFlywheelMotors (HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotor.class, FLYWHEEL_NAME);
        flywheelMotor.setDirection(FLYWHEEL_DIRECTION);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initLauncherSpine (HardwareMap hardwareMap) {
        launcherSpine = hardwareMap.get(Servo.class, LAUNCHER_SPINE_NAME);
        launcherSpine.setPosition(SPINE_POLLEN_POSITION);
    }

    private void initPinpoint (HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        pinpoint.setOffsets(FORWARD_POD_Y_MM, STRAFE_POD_X_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(PODS);
        pinpoint.setEncoderDirections(FORWARD_ENCODER_DIRECTION, STRAFE_ENCODER_DIRECTION);

        /*
         * Reset position and IMU at the start of every match / OpMode.
         *
         * In Iterative OpMode Style, any IMU setting time should be handled
         * explicitly in the init_loop() rather than with a blocking sleep here.
         * This keeps the hardware class lightweight during initialization.
         */
        pinpoint.resetPosAndIMU();
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
     * PUBLIC INTAKE API
     * ===================================================== */

    /**
     * Positive power collects. Negative power ejects.
     * The OpMode decides the sign. This method only applies power.
     */
    public void setIntakePower(double power) {
        intakeMotor.setPower(Range.clip(power, -1.0, 1.0));
    }

    /**
     * Positive power kicks the piece out. Negative power runs the kicker backward.
     */
    public void setKickerPower(double power) {
        kickerMotor.setPower(Range.clip(power, -1.0, 1.0));
    }

    /**
     * Collect wins over eject. Kick wins over kick-reverse.
     * Each requested motor runs at its fixed power while the button is held.
     */
    public void runIntake(boolean collect, boolean eject, boolean kick, boolean kickReverse) {
        double intakePower = 0.0;
        if (collect) {
            intakePower = INTAKE_POWER;
        } else if (eject) {
            intakePower = -INTAKE_POWER;
        }
        double kickerPower = 0.0;
        if (kick) {
            kickerPower = KICKER_POWER;
        } else if (kickReverse) {
            kickerPower = -KICKER_POWER;
        }
        setIntakePower(intakePower);
        setKickerPower(kickerPower);
    }

    public void stopIntake() {
        setIntakePower(0.0);
        setKickerPower(0.0);
    }

    public void setFlywheelPower(double power) {flywheelMotor.setPower(Range.clip(power, -1.0, 1.0));}

    public void runFlywheel(boolean hiveLaunch, boolean flowerLaunch, boolean fixJam) {
        double flywheelPower = 0.0;
        if (hiveLaunch) {
            flywheelPower = FLYWHEEL_POWER;
        } else if (flowerLaunch) {
            flywheelPower = 0.5 * FLYWHEEL_POWER;
        } else if (fixJam) {
            flywheelPower = -FLYWHEEL_POWER;
        }
        setFlywheelPower(flywheelPower);
    }

    public void stopFlywheel() {
        setFlywheelPower(0.0);
    }

    public void setLauncherSpinePosition(double position) {
        launcherSpine.setPosition(Range.clip(position, 0.0, 1.0));
    }

    /**
     * Nectar wins over pollen. Releasing both buttons leaves the servo at its last position.
     */
    public void runLauncherSpine(boolean nectar, boolean pollen) {
        if (nectar) {
            setLauncherSpinePosition(SPINE_NECTAR_POSITION);
        } else if (pollen) {
            setLauncherSpinePosition(SPINE_POLLEN_POSITION);
        }
    }
    /* =====================================================
     * PUBLIC ODOMETRY / PINPOINT API
     * ===================================================== */

    /**
     * Updates the Pinpoint's internal pose calculations from the pods and IMU.
     * Must be called every loop iteration before reading pose data.
     */
    public void updatePose() {
        if (pinpoint != null) {
            pinpoint.update();
        }
    }

    public Pose2D getPose() {
        if (pinpoint != null) {
            return pinpoint.getPosition();
        }
        return new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
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
     * Resets the Pinpoint position and IMU heading to zero.
     * Call this at the start of autonomous or whenever re-zeroing is desired.
     */
    public void resetPose() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
        }
    }

    /**
     * Returns true if the Pinpoint has completed IMU calibration and is ready
     * to provide high-accuracy pose data.
     *
     * <p>In an Iterative OpMode, this can be polled inside init_loop() after
     * calling resetPose() when guaranteed IMU readiness is required before starting</p>
     */
    public boolean isPinpointReady() {
        if (pinpoint == null) return false;
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    /**
     * Triggers a fresh position + IMU recalibration on the Pinpoint.
     * Useful after disturbances or when re-zeroing is needed mid-match.
     */
    public void recalibratePinpoint() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
        }
    }

    /**
     * Returns the raw GoBildaPinpointDriver for advanced use cases.
     * Most students should use the higher-level methods above instead.
     */
    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    /* =====================================================
     * TELEMETRY HELPERS
     * ===================================================== */

    public void addIntakeTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Intake", "%.2f", intakeMotor.getPower());
        telemetry.addData("Kicker", "%.2f", kickerMotor.getPower());
        telemetry.addData("Flywheel", "%.2f", flywheelMotor.getPower());
        telemetry.addData("Spine", "%.2f", launcherSpine.getPosition());
    }

    public void addDriveTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Drive", "FL %.2f FR %.2f BL %.2f BR %.2f",
                frontLeftDrive.getPower(),
                frontRightDrive.getPower(),
                backLeftDrive.getPower(),
                backRightDrive.getPower());
    }

    public void addPoseTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry,
                                 DistanceUnit distUnit, AngleUnit angleUnit) {
        Pose2D pose = getPose();
        telemetry.addData("Pose", "X %.2f Y %.2f H %.1f",
                pose.getX(distUnit),
                pose.getY(distUnit),
                pose.getHeading(angleUnit));
    }
}