package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.drone.DroneSystem;

@TeleOp(name="TestDrone", group="Linear OpMode")
public class TestDrone extends LinearOpMode {
    private DroneSystem droneSystem;
    private GamepadEx gamepad;

    @Override
    public void runOpMode() {
        droneSystem = new DroneSystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {
            gamepad.readButtons();

            // update the drone system inputs
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                droneSystem.toggleState();
            }
            droneSystem.loop();
        }
    }
}
