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
    private MecanumDriveTrain mecanumDriveTrain;

    private HangSystem hangSystem;
    private DroneSystem droneSystem;

    private Intake intake;
    private GamepadEx gamepad;

    public Robot(HardwareMap hwMap, Gamepad gamepad) {
        mecanumDriveTrain = new MecanumDriveTrain(hwMap);
        intake = new Intake(hwMap);
        this.gamepad = new GamepadEx(gamepad);
        hangSystem = new HangSystem(hwMap);
        droneSystem = new DroneSystem(hwMap);
    }

    public void loop() {
        /* Get current button state
         * - One toggle button that will move the Intake from PICKUP to SCORE and back to
         *   PICKUP. [ButtonA]
         * - One toggle button that will OPEN/CLOSE the LEFT claw while Intake is in PICKUP
         *   state. [ButtonX]
         * - One toggle button that will OPEN/CLOSE the RIGHT claw while Intake is in PICKUP
         *   state. [ButtonY]
         * - One toggle button to move between HANG and RETRACTED states. [ButtonDPAD_LEFT]
         * - One toggle button to move between INIT and FLY states. [ButtonDPAD_RIGHT]
         */
        gamepad.readButtons();

        // Update the meccanum inputs and trigger loop to move the robot
        mecanumDriveTrain.updateInputs(
                gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX()
        );
        mecanumDriveTrain.loop();

        // update the intake inputs
        if (gamepad.wasJustPressed(GamepadKeys.Button.A)) intake.toggleState();
        if (gamepad.wasJustPressed(GamepadKeys.Button.X)) intake.toggleLeftClaw();
        if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) intake.toggleRightClaw();
        intake.loop();

        // update the hang system inputs
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) hangSystem.toggleState();
        hangSystem.loop();

        // update the drone system inputs
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) droneSystem.toggleState();
        droneSystem.loop();
    }
}
