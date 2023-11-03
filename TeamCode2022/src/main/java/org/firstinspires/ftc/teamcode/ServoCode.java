package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.CRServo;

@TeleOp
public class ServoCode extends LinearOpMode {

    @Override
    public void runOpMode() {
        double gripPosition;
        gripPosition = 1;

        CRServo claw = new CRServo(hardwareMap, "claw");

        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.x && gripPosition == 1) {
                claw.set(1.0);
                sleep(1000);
                gripPosition = 0;
            }

            if (gamepad1.x && gripPosition == 0) {
                claw.set(-1.0);
                sleep(1000);
                gripPosition = 1;
            }

        }
    }
}