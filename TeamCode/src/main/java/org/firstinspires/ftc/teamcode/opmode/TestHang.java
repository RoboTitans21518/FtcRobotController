package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

            Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
            telemetry.addLine("DPAD LEFT pressed");
            telemetry.update();

            if (gamepad.isDown(GamepadKeys.Button.DPAD_UP)) {
                telemetry.addLine("DPAD LEFT pressed");
                telemetry.update();
                hangSystem.up();
            }
            if (gamepad.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                telemetry.addLine("DPAD RIGHT pressed");
                telemetry.update();
                hangSystem.down();
            }
            hangSystem.loop();
        }
    }
}
