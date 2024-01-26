package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystem.drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystem.drone.DroneSystem;
import org.firstinspires.ftc.teamcode.subsystem.hang.HangSystem;
import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;

public class Robot {
    //private MecanumDriveTrain mecanumDriveTrain;

    private HangSystem hangSystem;
    private DroneSystem droneSystem;

    private Intake intake;
    private GamepadEx gamepad;

    public Robot(HardwareMap hwMap, Gamepad gamepad) {
        //mecanumDriveTrain = new MecanumDriveTrain(hwMap);
        intake = new Intake(hwMap);
        this.gamepad = new GamepadEx(gamepad);
        hangSystem = new HangSystem(hwMap);
        droneSystem = new DroneSystem(hwMap);
    }

    public void loop() {
        gamepad.readButtons();

        // Update the meccanum inputs
        //mecanumDriveTrain.updateInputs(
        //        gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX()
        //);
        //mecanumDriveTrain.loop();

        // update the intake inputs
        if (gamepad.getButton(GamepadKeys.Button.A)) {
            intake.setLeftClawState(Intake.ClawState.OPEN);
        } else if (gamepad.getButton(GamepadKeys.Button.B)) {
            intake.setLeftClawState(Intake.ClawState.CLOSED);
        }
        intake.loop();

        // update the hang system inputs
        if (gamepad.getButton(GamepadKeys.Button.X)) {
            hangSystem.setState(HangSystem.HangState.HANG);
        } else if (gamepad.getButton(GamepadKeys.Button.Y)) {
            hangSystem.setState(HangSystem.HangState.RETRACTED);
        }
        hangSystem.loop();

        // update the drone system inputs
        if (gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            droneSystem.setState(DroneSystem.DroneState.FLY);
        }
        droneSystem.loop();
    }
}
