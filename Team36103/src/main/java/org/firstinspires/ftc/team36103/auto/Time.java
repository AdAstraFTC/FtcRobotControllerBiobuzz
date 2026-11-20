/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team36103.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team36103.Hardware;

/**
 * Timed robot-centric drive, then one catapult shot.
 *
 * <p>Drive power goes through {@link Hardware#driveRobotCentric}. The shot
 * goes through {@link Hardware#runCatapult}, the same up-then-return sequence
 * teleop uses. This OpMode only chooses when the shot starts. Hardware owns
 * the stroke times and powers. The robot is stopped before the shot.</p>
 */
@Autonomous(name = "Time", group = "Robot")
public class Time extends LinearOpMode {

    private static final double FORWARD_POWER = 1.0;
    private static final double TURN_POWER = 1.0;

    private static final double LEG1_SECONDS = 1.5;
    private static final double LEG2_SECONDS = 0.3;
    private static final double LEG3_SECONDS = 0.5;
    private static final double LEG4_SECONDS = 0.3;

    private Hardware hardware;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        hardware.stopDrive();
        hardware.stopCatapult();

        telemetry.addLine("Forward 1.5s, spin right 0.3s, forward 0.5s, spin right 0.3s, 1-ball shot.");
        telemetry.update();

        waitForStart();

        try {
            driveFor(FORWARD_POWER, 0.0, 0.0, LEG1_SECONDS, "Forward");
            driveFor(0.0, 0.0, TURN_POWER, LEG2_SECONDS, "Spin right");
            driveFor(FORWARD_POWER, 0.0, 0.0, LEG3_SECONDS, "Forward");
            driveFor(0.0, 0.0, TURN_POWER, LEG4_SECONDS, "Spin right");
            // Arguments are 4-ball, 3-ball, 2-ball, 1-ball. The first true value wins.
            fireShot(false, false, false, true);
        } finally {
            hardware.stopDrive();
            hardware.stopCatapult();
        }
    }

    /**
     * Holds one robot-centric command until {@code seconds} elapses.
     * The catapult stays in hold for the whole leg.
     */
    private void driveFor(double axial, double lateral, double yaw, double seconds, String leg) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {
            hardware.driveRobotCentric(axial, lateral, yaw);
            hardware.runCatapult(false, false, false, false, false);
            telemetry.addData("Path", "%s  %.1f s", leg, runtime.seconds());
            hardware.addDriveTelemetry(telemetry);
            telemetry.update();
        }
        hardware.stopDrive();
    }

    /**
     * One rising edge starts the shot. Later calls keep the button released so
     * Hardware can finish the up stroke and the return on its own clock.
     */
    private void fireShot(boolean shoot4, boolean shoot3, boolean shoot2, boolean shoot1) {
        hardware.stopDrive();
        hardware.runCatapult(shoot4, shoot3, shoot2, shoot1, false);
        while (opModeIsActive() && hardware.isCatapultRunning()) {
            hardware.runCatapult(false, false, false, false, false);
            telemetry.addData("Path", "Shot");
            hardware.addMechanismTelemetry(telemetry);
            telemetry.update();
        }
    }
}