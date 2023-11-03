package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Config
@TeleOp
public class SlideCodePosition extends LinearOpMode {

    static final boolean FIELD_CENTRIC = false;

    @Override
    public void runOpMode() {
        // Slide
        int[] positionArray = {0, 300, 500, 900};
        int positionIndex = 0;

        ElapsedTime slideTimer = new ElapsedTime();

        MotorEx slide = new MotorEx(hardwareMap, "slide", Motor.GoBILDA.RPM_1150);
        DcMotorEx slideMotor = slide.motorEx;

        // PID controller for slide
        PIDFController pidfController = new PIDFController(.003, .0007, 0, 0.001);

        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setPower(1);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Left bumper go from 1000 -> 500 -> 300 -> 0
            // Right bumper go from 0 -> 300 -> 500 -> 1000
            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                if (gamepad1.left_bumper) {
                    if ((positionIndex - 1) < 0)
                        positionIndex = 0;
                    else
                        positionIndex = positionIndex - 1;
                }
                if (gamepad1.right_bumper) {
                    if ((positionIndex + 1) >= positionArray.length)
                        positionIndex = positionArray.length - 1;
                    else
                        positionIndex = positionIndex + 1;
                }
                int targetPosition = positionArray[positionIndex];

                // Set the target position
                double currentPosition = slideMotor.getCurrentPosition();
                double output = pidfController.calculate(currentPosition, targetPosition);
                slideMotor.setTargetPosition(targetPosition);
                slideMotor.setPower(output);

                slideTimer.reset();
                slideTimer.startTime();
                while (opModeIsActive() && slideMotor.isBusy() && (slideTimer.seconds() <= 2)) {
                    telemetry.addLine(String.format("\noutput=%f positionIndex=%d", output, positionIndex));
                    telemetry.addData("Target: ", targetPosition);
                    telemetry.addData("Current: ", slideMotor.getCurrentPosition());
                    telemetry.update();
                }
            }
        }

        slideMotor.setPower(0);
    }
}
