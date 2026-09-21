/* Copyright (c) 2026 Coach Chris Lemoine - Team00000 Mentor Bot
 *
 * Team00000 is a reference and demonstration platform, not a library for direct use.
 * Study this OpMode and recreate it in your own team's module.
 */

package org.firstinspires.ftc.team00000.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.team00000.pedroPathing.Constants;

/**
 * Simple autonomous: drive forward a fixed distance and stop.
 *
 * <p>This uses Pedro Pathing's {@link Follower}, which already owns the mecanum
 * motors and Pinpoint. Do not also construct {@code Hardware} in this OpMode —
 * that would initialize the same devices twice. Wiring still comes from
 * {@code Hardware.Config} through {@link Constants#createFollower}.</p>
 *
 * <p><b>Setup:</b> place the robot with a clear path ahead. Heading 0 is +X
 * (straight forward from the starting pose). This is robot-start-relative, not
 * a full FTC field coordinate auto.</p>
 */
@Configurable
@Autonomous(name = "Forward", group = "Autonomous")
public class Forward extends OpMode {

    /** Distance to drive, in inches. Edit here or live-tune from Panels. */
    public static double FORWARD_INCHES = 24.0;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    private Follower follower;
    private PathChain forward;
    private boolean finished;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
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
        Pose endPose = new Pose(FORWARD_INCHES, 0, startPose.getHeading());

        forward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        follower.activateAllPIDFs();
        follower.followPath(forward, true);
    }

    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy()) {
            finished = true;
        }

        telemetry.addData("State", finished ? "DONE" : "DRIVING");
        telemetry.addData("Target (in)", FORWARD_INCHES);
        addPoseTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
        }
    }

    private void addPoseTelemetry() {
        Pose pose = follower.getPose();
        telemetry.addData("Pose (in)", "X %.2f Y %.2f H %.1f deg",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading()));
    }
}
