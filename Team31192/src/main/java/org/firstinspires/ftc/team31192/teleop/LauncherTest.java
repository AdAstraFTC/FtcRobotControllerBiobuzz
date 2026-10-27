package org.firstinspires.ftc.team31192.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team31192.Hardware;

/**
 * Stationary pollen flywheel test. Record the set power that scores from each square.
 *
 * <p>A toggles the flywheel on and off at the set power. D-pad up adds 0.01 and
 * D-pad down subtracts 0.01. The spine stays at pollen. Either gamepad works.
 * The same controls work before START.</p>
 */
@TeleOp(name = "Launcher Test", group = "Test")
public class LauncherTest extends OpMode {

    /** One step on the driver station is 0.01 power. Stored as hundredths to avoid drift. */
    private static final int POWER_STEP_HUNDREDTHS = 1;

    private Hardware hardware;
    private int powerHundredths = (int) Math.round(Hardware.FLYWHEEL_POWER * 100.0);
    private boolean flywheelOn = false;
    private boolean upHeld = false;
    private boolean downHeld = false;
    private boolean toggleHeld = false;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
        hardware.setLauncherSpinePosition(Hardware.SPINE_POLLEN_POSITION);
        hardware.stopFlywheel();
    }

    @Override
    public void init_loop() {
        readControls();
        addTestTelemetry();
    }

    @Override
    public void loop() {
        readControls();
        addTestTelemetry();
    }

    @Override
    public void stop() {
        hardware.stopFlywheel();
    }

    private void readControls() {
        boolean up = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean down = gamepad1.dpad_down || gamepad2.dpad_down;
        boolean toggle = gamepad1.a || gamepad2.a;

        if (up && !down && !upHeld) {
            bumpPower(POWER_STEP_HUNDREDTHS);
        } else if (down && !up && !downHeld) {
            bumpPower(-POWER_STEP_HUNDREDTHS);
        }
        if (toggle && !toggleHeld) {
            flywheelOn = !flywheelOn;
        }

        upHeld = up;
        downHeld = down;
        toggleHeld = toggle;

        hardware.setLauncherSpinePosition(Hardware.SPINE_POLLEN_POSITION);
        if (flywheelOn) {
            hardware.setFlywheelPower(powerHundredths / 100.0);
        } else {
            hardware.stopFlywheel();
        }
    }

    private void bumpPower(int hundredths) {
        powerHundredths += hundredths;
        if (powerHundredths > 100) {
            powerHundredths = 100;
        } else if (powerHundredths < 0) {
            powerHundredths = 0;
        }
    }

    private void addTestTelemetry() {
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Set power", "%.2f", powerHundredths / 100.0);
        telemetry.addData("Spine", "pollen %.2f", Hardware.SPINE_POLLEN_POSITION);
        telemetry.addLine("A toggles flywheel. D-pad up/down changes power by 0.01.");
        telemetry.update();
    }
}
