package org.firstinspires.ftc.team36103;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    private GoBildaPinpointDriver pinpoint = null;
    private DcMotor intake = null;
    private DcMotor leftCatapult = null;
    private DcMotor rightCatapult = null;
    private Servo leftFlowerRamp = null;
    private Servo rightFlowerRamp = null;

    public enum CatapultMode { UP, DOWN, HOLD }

    private CatapultMode catapultMode = CatapultMode.HOLD;
    private boolean catapultRunning = false;
    private final ElapsedTime catapultTimer = new ElapsedTime();
    private double catapultActivePower = 0.0;
    private boolean previousUpButton = false;

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
    public static final String PINPOINT_NAME = "pinpoint";
    public static final String INTAKE_NAME = "intake";
    public static final String LEFT_CATAPULT_NAME = "leftCatapult";
    public static final String RIGHT_CATAPULT_NAME = "rightCatapult";
    public static final String LEFT_FLOWER_RAMP_NAME = "leftServo";
    public static final String RIGHT_FLOWER_RAMP_NAME = "rightServo";

    public static final DcMotor.Direction INTAKE_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction LEFT_CATAPULT_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RIGHT_CATAPULT_DIRECTION = DcMotor.Direction.REVERSE;

    /** Positive power collects. Forward on the intake motor is collect. */
    public static final double INTAKE_IN_POWER = 1.0;
    public static final double INTAKE_OUT_POWER = -1.0;

    /**
     * Gamepad triggers are analog. At or above this value counts as pressed.
     * Both teleops use this so the intake and catapult triggers match.
     */
    public static final double TRIGGER_THRESHOLD = 0.2;

    /** Shot powers are negative because positive power pivots the catapult down. */
    public static final double SHOOT_4_POWER = -1.00;
    public static final double SHOOT_3_POWER = -0.96;
    public static final double SHOOT_2_POWER = -0.89;
    public static final double SHOOT_1_POWER = -0.85;
    public static final double CATAPULT_DOWN_POWER = 1.0;
    public static final double CATAPULT_HOLD_POWER = 0.0;

    /** Milliseconds the shot power is applied before the automatic return. */
    public static final double CATAPULT_RUN_MS = 150;
    /** Milliseconds of down power after a shot. Increase if the arm does not seat. */
    public static final double CATAPULT_RETURN_MS = 200;

    /** Positional servo travel. Horn offsets are baked into the start angles. */
    public static final double SERVO_RANGE_DEG = 270.0;
    public static final double LEFT_SERVO_START_DEG = 240.0;
    public static final double RIGHT_SERVO_START_DEG = 30.0;
    public static final double SERVO_TRAVEL_DEG = 105.0;
    /** Intermediate angle that filters nectar away from the robot. */
    public static final double SERVO_MID_DEG = 50.0;

    public static final DcMotor.Direction FRONT_LEFT_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction FRONT_RIGHT_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction BACK_LEFT_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction BACK_RIGHT_DIRECTION = DcMotor.Direction.FORWARD;

    /*
     * PINPOINT POD OFFSETS (millimeters)
     *
     * Locations of the two odometry pods relative to the tracking point, not the
     * Pinpoint computer itself. Same pairing Team00000 uses:
     *   xOffset / forward pod Y: left +, right -
     *   yOffset / strafe pod X: forward +, back -
     * The forward pod increases when the robot moves forward.
     * The strafe pod increases when the robot moves left.
     */
    public static final double FORWARD_POD_Y_MM = -153.50;
    public static final double STRAFE_POD_X_MM = 56.00;
    public static final DistanceUnit PINPOINT_DISTANCE_UNIT = DistanceUnit.MM;
    public static final GoBildaOdometryPods PODS = GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static final EncoderDirection FORWARD_ENCODER_DIRECTION = EncoderDirection.FORWARD;
    public static final EncoderDirection STRAFE_ENCODER_DIRECTION = EncoderDirection.REVERSED;

    /* =====================================================
     * CONSTRUCTOR
     * ===================================================== */
    public Hardware(HardwareMap hardwareMap) {
        initDriveMotors(hardwareMap);
        initPinpoint(hardwareMap);
        initIntake(hardwareMap);
        initCatapult(hardwareMap);
        initFlowerRamps(hardwareMap);
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

        // Brake is more predictable than coast when holding heading or stopping for turns.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Pose comes from the Pinpoint, not the drive-motor encoders.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initPinpoint(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        // xOffset = forward pod Y (left +); yOffset = strafe pod X (forward +).
        pinpoint.setOffsets(FORWARD_POD_Y_MM, STRAFE_POD_X_MM, PINPOINT_DISTANCE_UNIT);
        pinpoint.setEncoderResolution(PODS);
        pinpoint.setEncoderDirections(FORWARD_ENCODER_DIRECTION, STRAFE_ENCODER_DIRECTION);

        /*
         * Reset position and IMU once at construction. Do not sleep here.
         * Poll isPinpointReady() from init_loop() and wait until the Driver
         * Station shows READY before pressing START.
         */
        pinpoint.resetPosAndIMU();
    }

    private void initIntake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, INTAKE_NAME);
        intake.setDirection(INTAKE_DIRECTION);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0.0);
    }

    /**
     * Catapult 1 forward and catapult 2 reverse both pivot the arm down.
     * Open-loop power matches the timed shot sequence. Float lets the arm coast
     * at hold power instead of fighting the mechanism.
     */
    private void initCatapult(HardwareMap hardwareMap) {
        leftCatapult = hardwareMap.get(DcMotor.class, LEFT_CATAPULT_NAME);
        rightCatapult = hardwareMap.get(DcMotor.class, RIGHT_CATAPULT_NAME);

        leftCatapult.setDirection(LEFT_CATAPULT_DIRECTION);
        rightCatapult.setDirection(RIGHT_CATAPULT_DIRECTION);

        leftCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftCatapult.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightCatapult.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setCatapultPower(CATAPULT_HOLD_POWER);
    }

    private void initFlowerRamps(HardwareMap hardwareMap) {
        leftFlowerRamp = hardwareMap.get(Servo.class, LEFT_FLOWER_RAMP_NAME);
        rightFlowerRamp = hardwareMap.get(Servo.class, RIGHT_FLOWER_RAMP_NAME);
        setFlowerRampAngles(LEFT_SERVO_START_DEG, RIGHT_SERVO_START_DEG);
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
     * Drives the robot field-relatively using the current Pinpoint heading.
     *
     * <p>"Forward" on the joystick is the heading-zero direction — the way the
     * robot faced when pose was last reset — not automatically FTC field +Y.</p>
     *
     * @param axial   Forward (+) / backward (-) in the heading-zero frame [-1.0, 1.0]
     * @param lateral Left (+) / right (-) strafe in the heading-zero frame [-1.0, 1.0]
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
     * PUBLIC ODOMETRY / PINPOINT API
     * ===================================================== */

    /**
     * Updates the Pinpoint pose from the pods and its IMU.
     * Call this every loop before reading pose or driving field-centrically.
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
     * Resets position and IMU heading to zero and starts IMU recalibration.
     * The robot must be stationary. Poll {@link #isPinpointReady()} from
     * {@code init_loop()} before using heading for field-centric drive.
     */
    public void resetPose() {
        pinpoint.resetPosAndIMU();
    }

    /** True when the Pinpoint IMU calibration is finished. */
    public boolean isPinpointReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    /** Same as {@link #resetPose()}. */
    public void recalibratePinpoint() {
        resetPose();
    }

    /** Raw driver for a known field pose via {@code setPosition}. */
    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    /* =====================================================
     * PUBLIC INTAKE, CATAPULT, AND FLOWER RAMP API
     * ===================================================== */

    /**
     * Positive power collects. Negative power ejects.
     */
    public void setIntakePower(double power) {
        intake.setPower(Range.clip(power, -1.0, 1.0));
    }

    /**
     * Eject wins when both are held. Otherwise the requested direction runs
     * at its fixed power while the button is held.
     */
    public void runIntake(boolean intakeIn, boolean intakeOut) {
        if (intakeIn && intakeOut) {
            intakeIn = false;
        }
        if (intakeIn) {
            setIntakePower(INTAKE_IN_POWER);
        } else if (intakeOut) {
            setIntakePower(INTAKE_OUT_POWER);
        } else {
            setIntakePower(0.0);
        }
    }

    public void stopIntake() {
        setIntakePower(0.0);
    }

    /** Both catapult motors receive the same power. Directions make that pivot together. */
    public void setCatapultPower(double power) {
        double clipped = Range.clip(power, -1.0, 1.0);
        leftCatapult.setPower(clipped);
        rightCatapult.setPower(clipped);
    }

    /**
     * A new shot request starts a timed up stroke, then an automatic return.
     * Down cancels a new shot and, when no sequence is running, pivots down
     * while held. Four-ball wins over three, then two, then one.
     */
    public void runCatapult(boolean shoot4, boolean shoot3, boolean shoot2,
                            boolean shoot1, boolean down) {
        if (down) {
            shoot4 = false;
            shoot3 = false;
            shoot2 = false;
            shoot1 = false;
        }

        boolean upButtonPressed = shoot4 || shoot3 || shoot2 || shoot1;
        boolean newUpPress = upButtonPressed && !previousUpButton;

        if (newUpPress && !catapultRunning) {
            catapultRunning = true;
            catapultTimer.reset();
            if (shoot4) {
                catapultActivePower = SHOOT_4_POWER;
            } else if (shoot3) {
                catapultActivePower = SHOOT_3_POWER;
            } else if (shoot2) {
                catapultActivePower = SHOOT_2_POWER;
            } else if (shoot1) {
                catapultActivePower = SHOOT_1_POWER;
            } else {
                catapultActivePower = CATAPULT_HOLD_POWER;
            }
        }

        if (catapultRunning) {
            if (catapultTimer.milliseconds() < CATAPULT_RUN_MS) {
                catapultMode = CatapultMode.UP;
                setCatapultPower(catapultActivePower);
            } else if (catapultTimer.milliseconds() < CATAPULT_RUN_MS + CATAPULT_RETURN_MS) {
                catapultMode = CatapultMode.DOWN;
                setCatapultPower(CATAPULT_DOWN_POWER);
            } else {
                catapultRunning = false;
                catapultMode = CatapultMode.HOLD;
                setCatapultPower(CATAPULT_HOLD_POWER);
            }
        } else if (down) {
            catapultMode = CatapultMode.DOWN;
            setCatapultPower(CATAPULT_DOWN_POWER);
        } else {
            catapultMode = CatapultMode.HOLD;
            setCatapultPower(CATAPULT_HOLD_POWER);
        }

        previousUpButton = upButtonPressed;
    }

    public void stopCatapult() {
        catapultRunning = false;
        catapultMode = CatapultMode.HOLD;
        catapultActivePower = CATAPULT_HOLD_POWER;
        setCatapultPower(CATAPULT_HOLD_POWER);
    }

    /** True during the shot stroke and the return stroke. */
    public boolean isCatapultRunning() {
        return catapultRunning;
    }

    /**
     * Down wins over up, and up wins over the nectar mid position.
     * Releasing every button leaves the ramps where they are.
     * Holding all three treats that as up.
     */
    public void runFlowerRamp(boolean up, boolean down, boolean mid) {
        if (down && up && mid) {
            down = false;
        }
        if (down) {
            setFlowerRampAngles(LEFT_SERVO_START_DEG - SERVO_TRAVEL_DEG,
                    RIGHT_SERVO_START_DEG + SERVO_TRAVEL_DEG);
        } else if (up) {
            setFlowerRampAngles(LEFT_SERVO_START_DEG, RIGHT_SERVO_START_DEG);
        } else if (mid) {
            setFlowerRampAngles(LEFT_SERVO_START_DEG - SERVO_MID_DEG,
                    RIGHT_SERVO_START_DEG + SERVO_MID_DEG);
        }
    }

    public void stopMechanisms() {
        stopIntake();
        stopCatapult();
    }

    private static double angleToServo(double angleDeg) {
        return Range.clip(angleDeg / SERVO_RANGE_DEG, 0.0, 1.0);
    }

    private void setFlowerRampAngles(double leftDeg, double rightDeg) {
        leftFlowerRamp.setPosition(angleToServo(leftDeg));
        rightFlowerRamp.setPosition(angleToServo(rightDeg));
    }

    /* =====================================================
     * TELEMETRY HELPERS
     * ===================================================== */

    public void addMechanismTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Intake", "%.2f", intake.getPower());
        telemetry.addData("Catapult", "%s  power %.2f", catapultMode, catapultActivePower);
        telemetry.addData("Flower ramp", "L %.3f  R %.3f",
                leftFlowerRamp.getPosition(),
                rightFlowerRamp.getPosition());
    }

    public void addDriveTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        telemetry.addData("Drive", "FL %.2f FR %.2f BL %.2f BR %.2f",
                frontLeftDrive.getPower(),
                frontRightDrive.getPower(),
                backLeftDrive.getPower(),
                backRightDrive.getPower());
    }

    public void addPoseTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry,
                                 DistanceUnit distUnit,
                                 AngleUnit angleUnit) {
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
    public void addInitTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        telemetry.addData("Pinpoint", isPinpointReady() ? "READY - press START" : status.toString());
        addPoseTelemetry(telemetry, DistanceUnit.MM, AngleUnit.DEGREES);
    }
}
