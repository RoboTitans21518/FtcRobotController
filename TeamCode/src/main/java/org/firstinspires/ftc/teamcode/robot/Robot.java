package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystem.drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;

public class Robot {
    //private MecanumDriveTrain mecanumDriveTrain;
    private Intake intake;
    private GamepadEx gamepad;

    public Robot(HardwareMap hwMap, Gamepad gamepad) {
        //mecanumDriveTrain = new MecanumDriveTrain(hwMap);
        intake = new Intake(hwMap);
        this.gamepad = new GamepadEx(gamepad);
    }

    public void loop() {
        gamepad.readButtons();

        //mecanumDriveTrain.updateInputs(
        //        gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX()
        //);
        //mecanumDriveTrain.loop();

        if (gamepad.getButton(GamepadKeys.Button.A)) {
            intake.setLeftClawState(Intake.ClawState.OPEN);
        } else if (gamepad.getButton(GamepadKeys.Button.B)) {
            intake.setLeftClawState(Intake.ClawState.CLOSED);
        }
        intake.loop();
    }
}
