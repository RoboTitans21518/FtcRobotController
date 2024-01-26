package org.firstinspires.ftc.teamcode.subsystem.hang;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangSystem {
    private Motor hangMotor;
    private HangState state;

    public void setState(HangState hang) {
        this.state = state;
    }

    public enum HangState {
        RETRACTED,
        HANG
    }

    public HangSystem(HardwareMap hwMap) {
        hangMotor = new Motor(hwMap, "hang", Motor.GoBILDA.RPM_312);
        state = HangState.RETRACTED;
    }

    public void loop() {
        // React to state
    }
}
