package org.firstinspires.ftc.teamcode.subsystem.drone;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.hang.HangSystem;

public class DroneSystem {
    private Servo drone;
    private DroneState state;
    private double position;

    public void setState(DroneState state) {
        this.state = state;
    }

    public enum DroneState {
        INIT,
        FLY
    }

    public DroneSystem(HardwareMap hwMap) {
        drone = hwMap.get(Servo.class, "drone");
        state = DroneState.INIT;
        position = drone.getPosition();
    }

    public void loop() {
        // React to state
    }
}
