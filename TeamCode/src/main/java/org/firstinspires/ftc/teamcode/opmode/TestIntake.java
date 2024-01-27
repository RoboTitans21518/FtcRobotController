package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;

@TeleOp(name="TestIntake", group="Linear OpMode")
public class TestIntake extends LinearOpMode {
    private Intake intake;
    private GamepadEx gamepad;

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);
        gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {
            gamepad.readButtons();

            if (gamepad.wasJustPressed(GamepadKeys.Button.A)) intake.toggleState();
            if (gamepad.wasJustPressed(GamepadKeys.Button.X)) intake.toggleLeftClaw();
            if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) intake.toggleRightClaw();
            intake.loop();
        }
    }
}
