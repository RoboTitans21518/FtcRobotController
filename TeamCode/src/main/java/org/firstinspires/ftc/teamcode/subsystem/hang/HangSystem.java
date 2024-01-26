package org.firstinspires.ftc.teamcode.subsystem.hang;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.intake.Intake;

/*
 * The HANG system has a motor controlling the vertical hang.
 *
 * Starts in RETRACTED state. Extends and goes to HANG state. Should move
 * back to RETRACTED state to hang. And then finally needs to go back to
 * HANG to release the hang. And back to RETRACTED state to complete the
 * hang process.
 *
 * For Teleop we will need a button to toggle between HANG and RETRACTED
 * states.
 * - One toggle button to move between HANG and RETRACTED states. [ButtonDPAD_LEFT]
 */
public class HangSystem {
    private Motor hangMotor;
    private HangState state;

    // TODO: Tune these values
    private final double RETRACTED_POSITION = 100;
    private final double HANG_POSITION = 500;

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

    private void moveHangToPosition(double hangPosition) {

    }

    public void toggleState() {
        if (state == HangState.RETRACTED) state = HangState.HANG;
        if (state == HangState.HANG) state = HangState.RETRACTED;
    }

    public void loop() {
        // React to state
        switch (state) {
            case HANG:
                moveHangToPosition(HANG_POSITION);
                break;
            case RETRACTED:
                moveHangToPosition(RETRACTED_POSITION);
                break;
            default:
                break;
        }
    }
}
