package org.firstinspires.ftc.team00000.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team00000.Hardware;

/**
 * Pedro Pathing follower settings. Drivetrain names, motor directions, and
 * Pinpoint geometry live in {@link Hardware.Config} so teleop and autonomous
 * share one source of truth.
 */
@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.71645); // kilograms

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants() {
        return new MecanumConstants()
                .maxPower(Hardware.Config.DRIVE_MAX_POWER)
                .rightFrontMotorName(Hardware.Config.FRONT_RIGHT_NAME)
                .rightRearMotorName(Hardware.Config.BACK_RIGHT_NAME)
                .leftRearMotorName(Hardware.Config.BACK_LEFT_NAME)
                .leftFrontMotorName(Hardware.Config.FRONT_LEFT_NAME)
                .leftFrontMotorDirection(Hardware.Config.FRONT_LEFT_DIRECTION)
                .leftRearMotorDirection(Hardware.Config.BACK_LEFT_DIRECTION)
                .rightFrontMotorDirection(Hardware.Config.FRONT_RIGHT_DIRECTION)
                .rightRearMotorDirection(Hardware.Config.BACK_RIGHT_DIRECTION);
    }

    public static PinpointConstants localizerConstants() {
        return new PinpointConstants()
                .forwardPodY(Hardware.Config.FORWARD_POD_Y_MM)
                .strafePodX(Hardware.Config.STRAFE_POD_X_MM)
                .distanceUnit(Hardware.Config.PINPOINT_DISTANCE_UNIT)
                .hardwareMapName(Hardware.Config.PINPOINT_NAME)
                .encoderResolution(Hardware.Config.PODS)
                .forwardEncoderDirection(Hardware.Config.FORWARD_ENCODER_DIRECTION)
                .strafeEncoderDirection(Hardware.Config.STRAFE_ENCODER_DIRECTION);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants())
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants())
                .build();
    }
}
