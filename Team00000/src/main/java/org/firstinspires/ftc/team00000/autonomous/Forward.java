/* Copyright (c) 2026 Coach Chris Lemoine - Team00000 Mentor Bot
 *
 * Team00000 is a reference and demonstration platform, not a library for direct use.
 * Study this OpMode and recreate it in your own team's module.
 */

package org.firstinspires.ftc.team00000.autonomous;

import static com.pedropathing.api.Paths.line;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team00000.pedroPathing.Constants;

/**
 * Simple autonomous: drive forward a fixed distance and stop.
 *
 * <p>Uses Pedro Pathing 3's {@link Follower}, which owns the mecanum motors
 * and Pinpoint. Do not also construct {@code Hardware} here. Wiring still
 * comes from {@code Hardware.Config} through {@link Constants#create}.</p>
 *
 * <p>Place the robot with a clear path ahead. Heading 0 is +X from the
 * starting pose (robot-start-relative, not a full field auto).</p>
 */
@Configurable
@Autonomous(name = "Forward", group = "Autonomous")
@SuppressWarnings("unused") // Instantiated by the FTC SDK from the @Autonomous annotation.
public class Forward extends OpMode {

    /** Distance to drive, in inches. Edit here or live-tune from Panels. */
    public static double FORWARD_INCHES = 24.0;

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
    }

    @Override
    public void init_loop() {
        follower.update();

        telemetry.addLine("Clear " + FORWARD_INCHES + " in ahead of the robot.");
        telemetry.addLine("Press START to drive forward and hold.");
        addPoseTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        Pose endPose = new Pose(FORWARD_INCHES, 0, 0);
        Path forward = line(Pose.zero(), endPose).constant(0);

        follower.holdEnd.set(true);
        follower.follow(forward);
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("State", follower.isBusy() ? "DRIVING" : "DONE");
        telemetry.addData("Target (in)", FORWARD_INCHES);
        addPoseTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
        }
    }

    private void addPoseTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Pose (in)", "X %.2f Y %.2f H %.1f deg",
                pose.x(),
                pose.y(),
                Math.toDegrees(pose.heading()));
    }
}
