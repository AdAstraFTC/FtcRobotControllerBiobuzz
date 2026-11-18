package org.firstinspires.ftc.team36103.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team36103.Hardware;

@TeleOp(name = "RobotCentric", group = "TeleOp")
public class

RobotCentric extends OpMode {

    private Hardware hardware;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
    }

    @Override
    public void init_loop() {
        hardware.updatePose();
        hardware.addInitTelemetry(telemetry);
        telemetry.addData("Mode", "Robot Centric");
        telemetry.update();
    }

    @Override
    public void loop(){
        hardware.updatePose();

        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        hardware.driveRobotCentric(axial, lateral, yaw);
        runIntake();
        runCatapult();
        runFlowerRamp();

        // Telemetry
        hardware.addDriveTelemetry(telemetry);
        hardware.addMechanismTelemetry(telemetry);
        hardware.addPoseTelemetry(telemetry, DistanceUnit.MM, AngleUnit.DEGREES);
        telemetry.addData("Mode", "Robot Centric");
        telemetry.addData("Pinpoint", hardware.isPinpointReady() ? "READY" : "NOT READY");
        telemetry.update();
    }

    @Override
    public void stop() {
        hardware.stopMechanisms();
    }

    /** Gamepad 1: left trigger collects, left bumper ejects. Eject wins if both are held. */
    private void runIntake() {
        hardware.runIntake(
                gamepad1.left_trigger > Hardware.TRIGGER_THRESHOLD,
                gamepad1.left_bumper);
    }

    /**
     * Gamepad 1: right trigger 4-ball, Y 3-ball, B 2-ball, A 1-ball.
     * Right bumper pivots down and cancels a new shot.
     */
    private void runCatapult() {
        hardware.runCatapult(
                gamepad1.right_trigger > Hardware.TRIGGER_THRESHOLD,
                gamepad1.y,
                gamepad1.b,
                gamepad1.a,
                gamepad1.right_bumper);
    }

    /** Gamepad 1: dpad up raises the ramps, dpad down lowers them, dpad left is nectar mid. */
    private void runFlowerRamp() {
        hardware.runFlowerRamp(
                gamepad1.dpad_up,
                gamepad1.dpad_down,
                gamepad1.dpad_left);
    }
}