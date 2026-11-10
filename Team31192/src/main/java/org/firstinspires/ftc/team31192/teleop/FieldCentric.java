package org.firstinspires.ftc.team31192.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team31192.Hardware;

@TeleOp(name = "FieldCentric", group = "TeleOp")
public class FieldCentric extends OpMode {

    private Hardware hardware;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
        hardware.resetPose();
    }

    @Override
    public void loop(){
        hardware.updatePose();

        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        hardware.driveFieldCentric(axial, lateral, yaw);
        runIntake();
        runFlywheel();
        runLauncherSpine();
        runRamp();

        // Telemetry
        hardware.addDriveTelemetry(telemetry);
        hardware.addIntakeTelemetry(telemetry);
        hardware.addPoseTelemetry(telemetry, DistanceUnit.MM, AngleUnit.DEGREES);
        telemetry.addData("Mode", "Field Centric");
        telemetry.update();
    }

    @Override
    public void stop() {
        hardware.stopIntake();
        hardware.stopFlywheel();
    }

    private void runIntake() {
        hardware.runIntake(
                gamepad2.right_bumper,
                gamepad2.left_bumper,
                gamepad2.a,
                gamepad2.b);
    }

    /** Gamepad 2: Y full power, X half power, dpad down reverse. Hive wins, then flower, then jam. */
    private void runFlywheel() {
        hardware.runFlywheel(
                gamepad2.y,
                gamepad2.x,
                gamepad2.dpad_down);
    }

    /** Gamepad 2: dpad up nectar, dpad left pollen. Nectar wins. Releasing both holds position. */
    private void runLauncherSpine() {
        hardware.runLauncherSpine(
                gamepad2.dpad_up,
                gamepad2.dpad_left);
    }

    public void runRamp () {
        hardware.runRamp(
                gamepad2.right_trigger > 0.5, gamepad2.left_trigger > 0.5);
    }
}