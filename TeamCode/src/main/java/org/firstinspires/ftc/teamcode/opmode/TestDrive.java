package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drive.MecanumDriveTrain;

@Config
@TeleOp(name="TestDrive", group="Linear OpMode")
public class TestDrive extends LinearOpMode {
    private MecanumDriveTrain drive;
    private GamepadEx gamepad;

    public static double clawPosition = 0;
    @Override
    public void runOpMode() {
        drive = new MecanumDriveTrain(hardwareMap);
        gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {
            gamepad.readButtons();

            drive.updateInputs(
                    gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX()
            );
            drive.loop();
        }
    }
}
