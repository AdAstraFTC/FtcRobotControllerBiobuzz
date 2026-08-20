package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team00000.Hardware;

@TeleOp(name = "RobotCentric", group = "TeleOp")
public class RobotCentric extends OpMode {

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

        hardware.driveRobotCentric(axial, lateral, yaw);

        // Telemetry
        hardware.addDriveTelemetry(telemetry);
        hardware.addPoseTelemetry(telemetry, DistanceUnit.MM, AngleUnit.DEGREES);
        telemetry.addData("Mode", "Robot Centric");
        telemetry.update();
    }
}