package org.firstinspires.ftc.teamcode.subsystem.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Intake {
    private Servo leftClaw;
    private Servo rightClaw;
    private Servo rotator;

    private ClawState leftClawState;
    private ClawState rightClawState;
    private RotateState rotateState;

    private double leftClawPosition;
    private double rightClawPosition;
    private double rotatePosition;

    public enum ClawState {
        CLOSED,
        OPEN
    }

    public enum RotateState {
        INIT,
        FLAT,
        SCORE
    }

    public Intake(HardwareMap hwMap) {
        leftClaw = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");
        rotator = hwMap.get(Servo.class, "rotator");
        leftClawState = ClawState.CLOSED;
        rightClawState = ClawState.CLOSED;
        rotateState = RotateState.FLAT;
        leftClawPosition = leftClaw.getPosition();
        rightClawPosition = rightClaw.getPosition();
        rotatePosition = rotator.getPosition();
    }

    public void setLeftClawState(ClawState state) {
        leftClawState = state;
    }

    public void setRightClawState(ClawState state) {
        rightClawState = state;
    }

    public void setRotateState(RotateState state) {
        rotateState = state;
    }

    public void loop() {
        // Handle states
        if (leftClawState == ClawState.CLOSED) {
            leftClaw.setPosition(.5);
        } else if (leftClawState == ClawState.OPEN) {
            leftClaw.setPosition(0);
        }
    }

}
