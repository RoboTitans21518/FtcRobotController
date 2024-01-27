package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.hang.HangSystem;

@TeleOp(name="TestHang", group="Linear OpMode")
public class TestHang extends LinearOpMode {
    private HangSystem hangSystem;
    private GamepadEx gamepad;

    @Override
    public void runOpMode() {
        hangSystem = new HangSystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {
            gamepad.readButtons();

            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) hangSystem.toggleState();
            hangSystem.loop();
        }
    }
}
