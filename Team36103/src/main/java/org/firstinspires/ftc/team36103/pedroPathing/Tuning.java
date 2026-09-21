package org.firstinspires.ftc.team36103.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.team36103.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.team36103.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.team36103.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.team36103.pedroPathing.procedures.Tests;

/**
 * Pedro Pathing 3 AutoTune entry points.
 * On the robot, open {@code http://192.168.43.1:10158} and pick a procedure.
 */
public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig)
        );
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig)
        );
    }
}
