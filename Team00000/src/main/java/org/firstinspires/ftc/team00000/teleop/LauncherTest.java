/* Copyright (c) 2026 Coach Chris Lemoine - Team00000 Mentor Bot
 *
 * Team00000 is a reference and demonstration platform, not a library for direct use.
 * Study this OpMode and recreate it in your own team's module.
 */

package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team00000.Hardware;

/**
 * Bench test for the launcher flywheel and 5-turn servo.
 *
 * <p>Does not drive. The flywheel stays coasting until the right trigger is held,
 * then it runs at {@link Hardware.Config#FLYWHEEL_POWER}. The motor encoder is not
 * used. A moves the servo to pollen and B moves it to nectar. Either gamepad
 * works. Releasing both leaves the servo where it is. The same controls work
 * before START. Change the positions and flywheel power from Panels on
 * {@link Hardware.Config}.</p>
 *
 * <p>The servo moves to pollen as soon as this OpMode is selected, because
 * {@link Hardware} commands that position at init.</p>
 */
@TeleOp(name = "Launcher Test", group = "Test")
@SuppressWarnings("unused") // Instantiated by the FTC SDK from the @TeleOp annotation.
public class LauncherTest extends OpMode {

    private Hardware hardware;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
    }

    @Override
    public void init_loop() {
        readControls();
        addControlTelemetry();
    }

    @Override
    public void loop() {
        readControls();
        addControlTelemetry();
    }

    private void readControls() {
        boolean pollen = gamepad1.a || gamepad2.a;
        boolean nectar = gamepad1.b || gamepad2.b;
        if (pollen) {
            hardware.setLauncherServoPosition(Hardware.Config.LAUNCHER_SERVO_POLLEN);
        } else if (nectar) {
            hardware.setLauncherServoPosition(Hardware.Config.LAUNCHER_SERVO_NECTAR);
        }

        double trigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
        if (trigger > 0.5) {
            hardware.setFlywheelPower(Hardware.Config.FLYWHEEL_POWER);
        } else {
            hardware.stopFlywheel();
        }
    }

    private void addControlTelemetry() {
        hardware.addLauncherTelemetry(telemetry);
        telemetry.addData("Gamepad1", "A %b  B %b  RT %.2f",
                gamepad1.a, gamepad1.b, gamepad1.right_trigger);
        telemetry.addData("Gamepad2", "A %b  B %b  RT %.2f",
                gamepad2.a, gamepad2.b, gamepad2.right_trigger);
        telemetry.addLine("A pollen 0.000. B nectar 0.075. Right trigger flywheel.");
        telemetry.update();
    }

    @Override
    public void stop() {
        hardware.stopFlywheel();
    }
}
