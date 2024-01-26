package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;

@TeleOp(name="TestIntake", group="Linear OpMode")
public class TestIntake extends LinearOpMode {
    private Intake intake = new Intake(hardwareMap);
    private GamepadEx gamepad = new GamepadEx(gamepad1);

    @Override
    public void runOpMode() {
        waitForStart();

        while (!isStopRequested()) {
            gamepad.readButtons();

            if (gamepad.getButton(GamepadKeys.Button.A)) intake.toggleState();
            if (gamepad.getButton(GamepadKeys.Button.X)) intake.toggleLeftClaw();
            if (gamepad.getButton(GamepadKeys.Button.Y)) intake.toggleRightClaw();
            intake.loop();
        }
    }
}
