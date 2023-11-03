package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp

public class DriverOpMode extends LinearOpMode {

    static final boolean FIELD_CENTRIC = false;

    @Override
    public void runOpMode() {
        // Slide
        int LOW = 1200;
        int MEDIUM = 2500;
        int RESET = 0;
        int MAX_WAIT_FOR_SLIDE = 2; // In seconds

        ElapsedTime slideTimer = new ElapsedTime();
        ElapsedTime slideButtonTimer = new ElapsedTime();
        int SLIDE_DEBOUNCE_TIME = 700; // In milliseconds

        MotorEx slide = new MotorEx(hardwareMap, "slide", Motor.GoBILDA.RPM_1150);
        DcMotorEx slideMotor = slide.motorEx;

        // PID controller for slide
        PIDFController pidfController = new PIDFController(.003, .0007, 0, 0.001);

        // Claw true = Close false = Open
        boolean gripPosition = true;
        SimpleServo claw = new SimpleServo(hardwareMap, "claw", 30, 60, AngleUnit.DEGREES);
        ElapsedTime clawButtonTimer = new ElapsedTime();
        int CLAW_DEBOUNCE_TIME = 700; // In milliseconds

        // Drive train
        MecanumDrive drive = new MecanumDrive(
                new MotorEx(hardwareMap, "frontL", Motor.GoBILDA.RPM_312),
                new MotorEx(hardwareMap, "frontR",  Motor.GoBILDA.RPM_312),
                new MotorEx(hardwareMap, "backL",  Motor.GoBILDA.RPM_312),
                new MotorEx(hardwareMap, "backR",  Motor.GoBILDA.RPM_312)
        );

        // Game pad
        GamepadEx driverOp = new GamepadEx(gamepad1);

        // Initialize slide
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setPower(1);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Left bumper go from 1000 -> 500 -> 300 -> 0
            // Right bumper go from 0 -> 300 -> 500 -> 1000
            if ((gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_down || gamepad1.dpad_right) &&
                    (slideButtonTimer.milliseconds() > SLIDE_DEBOUNCE_TIME)) {
                telemetry.addLine(String.format("\ndpad_up=%b, dpad_down=%b", gamepad1.dpad_up, gamepad1.dpad_down));
                telemetry.addLine(String.format("\ndpad_left=%b, dpad_right=%b", gamepad1.dpad_left, gamepad1.dpad_right));
                slideButtonTimer.reset();

                int targetPosition = 0;
                if (gamepad1.dpad_down) {
                    targetPosition = LOW;
                }
                if (gamepad1.dpad_left) {
                    targetPosition = MEDIUM;
                }
                if(gamepad1.dpad_up){
                    targetPosition = MEDIUM;
                }
                if (gamepad1.dpad_right) {
                    targetPosition = RESET;
                }

                // Set the target position
                double currentPosition = slideMotor.getCurrentPosition();
                double output = pidfController.calculate(currentPosition, targetPosition);
                slideMotor.setTargetPosition(targetPosition);
                slideMotor.setPower(output);

                slideTimer.reset();
                slideTimer.startTime();
                while (opModeIsActive() && slideMotor.isBusy() && (slideTimer.seconds() <= MAX_WAIT_FOR_SLIDE)) {
                    telemetry.addLine(String.format("\noutput=%f", output));
                    telemetry.addData("Target: ", targetPosition);
                    telemetry.addData("Current: ", slideMotor.getCurrentPosition());
                    telemetry.update();
                }
            }

            // Toggle `X` button grip to open and close

            if (gamepad1.right_bumper && (clawButtonTimer.milliseconds() > CLAW_DEBOUNCE_TIME)) {
                clawButtonTimer.reset();
//                gripPosition = !gripPosition;
//                telemetry.addLine(String.format("\ngamepadx=%b, gripposition=%b", gamepad1.x, gripPosition));
//                telemetry.addLine(String.format("\nangle=%f, position=%f, gripposition=%b",
//                        claw.getAngle(), claw.getPosition(), gripPosition));
//                telemetry.update();
                claw.setPosition(.32);
            }
            if(gamepad1.left_bumper && (clawButtonTimer.milliseconds() > CLAW_DEBOUNCE_TIME)){
                clawButtonTimer.reset();
                claw.setPosition(.47);
            }


            // Drive train
            double leftX = driverOp.getLeftX();
            double leftY = driverOp.getLeftY();
            double rightX = driverOp.getRightX();

            // reverse the trajectory of the robot because it originally went in the opposite direction.
            leftX = leftX * -1;
            leftY = leftY * -1;
            rightX = rightX * -1;

            // Too powerful, reducing the power.
            double powerDivisor = 2;

            leftX = leftX/powerDivisor;
            leftY = leftY/powerDivisor;
            rightX = rightX/powerDivisor;
            if (!FIELD_CENTRIC) {
                drive.driveRobotCentric(
                        leftX,
                        leftY,
                        rightX,
                        true
                );
            } else {
                drive.driveFieldCentric(
                        leftX,
                        leftY,
                        rightX,
                        90,   // gyro value passed in here must be in degrees
                        false
                );
            }
        }

        // Stop the motors, drive train, claw
        slideMotor.setPower(0);
        claw.disable();
        drive.stop();
    }
}
