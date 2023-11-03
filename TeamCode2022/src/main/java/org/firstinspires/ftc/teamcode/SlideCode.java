package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp

public class SlideCode extends LinearOpMode {

    static final boolean FIELD_CENTRIC = false;

    @Override
    public void runOpMode() {
        double kS = 1.0;
        double kG = 1.0;
        double kV = 1.0;
        double kA = 1.0;
        ElevatorFeedforward feedforward = new ElevatorFeedforward(
                kS, kG, kV, kA
        );
        MotorEx slide = new MotorEx(hardwareMap, "slide", Motor.GoBILDA.RPM_1150);
        slide.set(0);

        waitForStart();

        double velocity = 1000;
        while (!isStopRequested()) {
            double curVelocity = slide.getVelocity();
            telemetry.addLine(String.format("\n1 gamepad.a=%b, gamepad.b=%b, gamepad.y=%b velocity=%f curVelocity=%f",
                    gamepad1.a, gamepad1.b, gamepad1.y, velocity, curVelocity));
            telemetry.update();
            if (gamepad1.y) slide.set(0);

            if (gamepad1.b || gamepad1.a) {
                if (gamepad1.a) {
                    velocity = velocity - 1000;
                }
                if (gamepad1.b) {
                    velocity = velocity + 1000;
                }
                //slide.set(feedforward.calculate(velocity, 20));
                slide.setVelocity(velocity);
            }
        }
    }

}

