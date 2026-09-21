package org.firstinspires.ftc.team00000.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team00000.Hardware;

/**
 * Pedro Pathing 3 constants. Drivetrain names, motor directions, and Pinpoint
 * geometry come from {@link Hardware.Config} so teleop and autonomous stay aligned.
 *
 * <p>Foresight uses library defaults until you run ForesightTuner in AutoTune
 * ({@code http://192.168.43.1:10158}) and paste the generated config here.</p>
 */
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(Hardware.Config.FRONT_LEFT_NAME);
        c.frontRightName.set(Hardware.Config.FRONT_RIGHT_NAME);
        c.backLeftName.set(Hardware.Config.BACK_LEFT_NAME);
        c.backRightName.set(Hardware.Config.BACK_RIGHT_NAME);
        c.frontLeftDirection.set(Hardware.Config.FRONT_LEFT_DIRECTION);
        c.frontRightDirection.set(Hardware.Config.FRONT_RIGHT_DIRECTION);
        c.backLeftDirection.set(Hardware.Config.BACK_LEFT_DIRECTION);
        c.backRightDirection.set(Hardware.Config.BACK_RIGHT_DIRECTION);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(Hardware.Config.PINPOINT_NAME);
        c.podType.set(Hardware.Config.PODS);
        c.xPodOffset.set(Hardware.Config.FORWARD_POD_Y_MM);
        c.yPodOffset.set(Hardware.Config.STRAFE_POD_X_MM);
        c.xPodDirection.set(Hardware.Config.FORWARD_ENCODER_DIRECTION);
        c.yPodDirection.set(Hardware.Config.STRAFE_ENCODER_DIRECTION);
        c.offsetUnits.set(DistanceUnit.MM);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return create(hardwareMap);
    }
}
