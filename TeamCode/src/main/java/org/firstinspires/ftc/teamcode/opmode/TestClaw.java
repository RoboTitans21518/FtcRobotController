package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="TestClaw", group="Linear OpMode")
public class TestClaw extends LinearOpMode {
    private Servo claw;

    public static double clawPosition = 0.4;
    @Override
    public void runOpMode() {
        claw = hardwareMap.get(Servo.class, "drone");

        waitForStart();

        while (!isStopRequested()) {
            claw.setPosition(clawPosition);
        }
    }
}
