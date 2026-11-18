package org.firstinspires.ftc.team36103.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team36103.Hardware;

/**
 * Pedro Pathing 3 follower for Team 36103.
 *
 * <p>Drive motor names, directions, and Pinpoint geometry come from {@link Hardware}.
 * Foresight coefficients stay here. Poses are in inches.</p>
 */
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(Hardware.FRONT_LEFT_NAME);
        c.frontRightName.set(Hardware.FRONT_RIGHT_NAME);
        c.backLeftName.set(Hardware.BACK_LEFT_NAME);
        c.backRightName.set(Hardware.BACK_RIGHT_NAME);
        c.frontLeftDirection.set(Hardware.FRONT_LEFT_DIRECTION);
        c.frontRightDirection.set(Hardware.FRONT_RIGHT_DIRECTION);
        c.backLeftDirection.set(Hardware.BACK_LEFT_DIRECTION);
        c.backRightDirection.set(Hardware.BACK_RIGHT_DIRECTION);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(Hardware.PINPOINT_NAME);
        c.podType.set(Hardware.PODS);
        // setOffsets(x, y): x is the forward pod's sideways offset, y is the strafe pod's forward offset.
        c.xPodOffset.set(Hardware.FORWARD_POD_Y_MM);
        c.yPodOffset.set(Hardware.STRAFE_POD_X_MM);
        c.xPodDirection.set(Hardware.FORWARD_ENCODER_DIRECTION);
        c.yPodDirection.set(Hardware.STRAFE_ENCODER_DIRECTION);
        c.offsetUnits.set(Hardware.PINPOINT_DISTANCE_UNIT);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(1);
        Controller secondaryTranslationalForward = Controller.proportional(1);
        Controller primaryTranslationalLateral = Controller.proportional(1);
        Controller secondaryTranslationalLateral = Controller.proportional(1);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
        c.coast.set(Controller.proportionalFeedforward(1));
        c.brake.set(Controller.proportionalFeedforward(1));
        c.headingFeedback.set(Controller.proportional(1));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0, 0));
        c.linearBrakeCoefficients.set(Matrix.diag(0, 0));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0, 0));
        c.maxAchievableForwardVelocity.set(1.0);
        c.maxAchievableStrafeVelocity.set(1.0);
        c.naturalForwardDeceleration.set(1.0);
        c.naturalStrafeDeceleration.set(1.0);
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
