package org.firstinspires.ftc.teamcode.subsystem.hang;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private DcMotorEx hangMotor;

    public HangSystem(HardwareMap hwMap) {
        hangMotor = hwMap.get(DcMotorEx.class, "hang");
    }

    public void up() {
        hangMotor.setPower(1);
        try {
            Thread.sleep(100);
        } catch (Exception e) {}
        hangMotor.setPower(0);
    }

    public void down() {
        hangMotor.setPower(-1);
        try {
            Thread.sleep(100);
        } catch (Exception e) {}
        hangMotor.setPower(0);
    }

    public void loop() {
    }
}
