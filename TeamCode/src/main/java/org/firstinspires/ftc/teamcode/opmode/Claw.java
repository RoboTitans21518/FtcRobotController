package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="CenterStage Claw", group="Linear OpMode")
public class Claw extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.a) {
                claw.setPosition(.75);
            } else if (gamepad1.b) {
                claw.setPosition(.25);
            } else {
                claw.setPosition(.5);
            }

            telemetry.addData("Claw:",  claw);
            telemetry.update();
        }
    }
}
