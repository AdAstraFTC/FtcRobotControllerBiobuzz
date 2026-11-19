package org.firstinspires.ftc.team36103.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team36103.Hardware;

/**
 * Runs one mecanum motor at a time through {@link Hardware}, using the same
 * names and directions as teleop. Lift the robot before pressing a button.
 */
@Disabled
@TeleOp(name = "Drive Motor Test", group = "Test")
public class DriveMotorTest extends OpMode {

    private static final double TEST_POWER = 0.30;

    private Hardware hardware;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
        telemetry.addLine("Lift the robot. Hold one dpad button.");
        telemetry.addLine("Up front left, right front right");
        telemetry.addLine("Down back right, left back left");
        telemetry.addLine("Left bumper reverses that wheel");
        telemetry.update();
    }

    @Override
    public void loop() {
        double power = gamepad1.left_bumper ? -TEST_POWER : TEST_POWER;
        double frontLeft = 0.0;
        double frontRight = 0.0;
        double backLeft = 0.0;
        double backRight = 0.0;
        String wheel = "none";
        String configName = "";

        if (gamepad1.dpad_up) {
            frontLeft = power;
            wheel = "front left";
            configName = Hardware.FRONT_LEFT_NAME;
        } else if (gamepad1.dpad_right) {
            frontRight = power;
            wheel = "front right";
            configName = Hardware.FRONT_RIGHT_NAME;
        } else if (gamepad1.dpad_down) {
            backRight = power;
            wheel = "back right";
            configName = Hardware.BACK_RIGHT_NAME;
        } else if (gamepad1.dpad_left) {
            backLeft = power;
            wheel = "back left";
            configName = Hardware.BACK_LEFT_NAME;
        }

        hardware.setDrivePower(frontLeft, frontRight, backLeft, backRight);

        hardware.addDriveTelemetry(telemetry);
        telemetry.addData("Wheel", wheel);
        telemetry.addData("Config", configName);
        telemetry.addData("Command", "%.2f", wheel.equals("none") ? 0.0 : power);
        telemetry.addLine("Wrong wheel spins: that config name is on the wrong port.");
        telemetry.addLine("Power shows and nothing spins: that motor or cable is dead.");
        telemetry.update();
    }

    @Override
    public void stop() {
        hardware.stopDrive();
    }
}
