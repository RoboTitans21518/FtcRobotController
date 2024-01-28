package org.firstinspires.ftc.teamcode.subsystem.drone;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.hang.HangSystem;

/*
 * The DRONE system has a servo controlling the trigger.
 *
 * Starts in INIT state. Moves to FLY state to trigger the drone.
 *
 * For Teleop we will need a button to trigger the drone.
 * - One toggle button to move between INIT and FLY states. [ButtonDPAD_RIGHT]
 */
public class DroneSystem {
    private Servo drone;
    private DroneState state;
    private double position;

    private final double INIT_POSITION = .55;
    private final double FLY_POSITION = .4;

    public enum DroneState {
        INIT,
        FLY
    }

    public DroneSystem(HardwareMap hwMap) {
        drone = hwMap.get(Servo.class, "drone");
        state = DroneState.INIT;
        position = drone.getPosition();
    }

    private void moveDroneToPosition(double dronePosition) {
        drone.setPosition(dronePosition);
    }

    public void toggleState() {
        if (state == DroneState.INIT) state = DroneState.FLY;
        if (state == DroneState.FLY) state = DroneState.INIT;
    }

    public void loop() {
        // React to state
        switch (state) {
            case FLY:
                moveDroneToPosition(FLY_POSITION);
                break;
            case INIT:
                moveDroneToPosition(INIT_POSITION);
                break;
            default:
                break;
        }
    }
}
