package org.firstinspires.ftc.teamcode.subsystem.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Intake has an ARM motor, rotator servo and a servo each for left and right claw. It
 * needs to follow the below sequence represented as states.
 * - Intake starts in an INIT state (droopy rotate servo and arm) and needs to be put
 *   in the PICKUP position to pickup pixels.
 * - After pixels are picked up. It needs to go to the SCORE position.
 * - And back to the PICKUP position after scoring.
 *
 * The ARM starts from rest and needs to move to a PICKUP and SCORE position.
 * - Starts in INIT and needs to be put in PICKUP
 * - Moves from PICKUP to SCORE and back.
 *
 * The ROTATOR servo
 * - Starts in INIT and needs to be put in PICKUP
 * - Moves from PICKUP to SCORE and back.
 *
 * The LEFT/RIGHT claws
 * - Starts in INIT position. And should already be setup to be in CLOSED position.
 * - Each claw can move from CLOSED to OPEN and back.
 *
 * For Teleop we need buttons to control the robot.
 * - One toggle button that will move the Intake from PICKUP to SCORE and back to
 *   PICKUP. [ButtonA]
 * - One toggle button that will OPEN/CLOSE the LEFT claw while Intake is in PICKUP
 *   state. [ButtonX]
 * - One toggle button that will OPEN/CLOSE the RIGHT claw while Intake is in PICKUP
 *   state. [ButtonY]
 */
@Config
public class Intake {
    private Motor armMotor;
    private Servo leftClaw;
    private Servo rightClaw;
    private Servo rotator;

    private ArmState armState;
    private ClawState leftClawState;
    private ClawState rightClawState;
    private RotateState rotateState;
    private IntakeState intakeState;

    private double leftClawPosition;
    private double rightClawPosition;
    private double rotatePosition;
    private double armPosition;

    // TODO: Tune these numbers
    public static double ARM_PICKUP_POSITION = .1;
    public static double ARM_SCORE_POSITION = .5;

    public static double LEFT_CLAW_OPEN_POSITION = 0.3;
    public static double LEFT_CLAW_CLOSE_POSITION = 0.1;
    public static double RIGHT_CLAW_OPEN_POSITION = 0.4;
    public static double RIGHT_CLAW_CLOSE_POSITION = 0.6;

    public static double ROTATOR_FLAT_POSITION = 0.2;
    public static double ROTATOR_SCORE_POSITION = 0.8;

    public enum ArmState {
        INIT,
        PICKUP,
        SCORE
    }

    public enum ClawState {
        INIT,
        CLOSED,
        OPEN
    }

    public enum RotateState {
        INIT,
        FLAT,
        SCORE
    }

    public enum IntakeState {
        INIT,
        PICKUP,
        SCORE
    }

    public enum Event {
        INIT_EVENT,
        INTAKE_TOGGLE,
        LEFT_CLAW_TOGGLE,
        RIGHT_CLAW_TOGGLE,
        ROTATOR_TOGGLE
    }

    public Intake(HardwareMap hwMap) {
        armMotor = new Motor(hwMap, "armMotor", Motor.GoBILDA.RPM_312);
        armMotor.setInverted(true);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        leftClaw = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");
        rotator = hwMap.get(Servo.class, "rotator");

        leftClawState = ClawState.INIT;
        rightClawState = ClawState.INIT;
        rotateState = RotateState.INIT;
        armState = ArmState.INIT;
        intakeState = IntakeState.INIT;

        leftClawPosition = leftClaw.getPosition();
        rightClawPosition = rightClaw.getPosition();
        rotatePosition = rotator.getPosition();
        armPosition = armMotor.getCurrentPosition();

        // Call the method to set INIT position
        handleIntakeState(intakeState, Event.INIT_EVENT);
        loop();
    }

    private void handleIntakeState(IntakeState current, Event event) {
        switch(event) {

            case INIT_EVENT:
                // Setup the intake system ready after power up
                armState = ArmState.PICKUP;
                leftClawState = ClawState.CLOSED;
                rightClawState = ClawState.CLOSED;
                rotateState = RotateState.FLAT;
                intakeState = IntakeState.PICKUP;
                break;
            case INTAKE_TOGGLE:
                // Move from PICKUP to SCORE and vice versa
                if (intakeState == IntakeState.SCORE) {
                    armState = ArmState.PICKUP;
                    rotateState = RotateState.FLAT;
                    intakeState = IntakeState.PICKUP;
                }
                if (intakeState == IntakeState.PICKUP) {
                    armState = ArmState.SCORE;
                    rotateState = RotateState.SCORE;
                    intakeState = IntakeState.SCORE;
                }
                break;
            default:
                return;
        }
    }

    private void handleLeftClawState(ClawState current, Event event) {
        switch (event) {
            case LEFT_CLAW_TOGGLE:
                // Handle toggle of left claw
                if (current == ClawState.CLOSED) {
                    leftClawState = ClawState.OPEN;
                }
                if (current == ClawState.OPEN) {
                    leftClawState = ClawState.CLOSED;
                }
            default:
                break;
        }
    }

    private void handleRightClawState(ClawState current, Event event) {
        switch (event) {
            case RIGHT_CLAW_TOGGLE:
                // Handle toggle of left claw
                if (current == ClawState.CLOSED) {
                    rightClawState = ClawState.OPEN;
                }
                if (current == ClawState.OPEN) {
                    rightClawState = ClawState.CLOSED;
                }
            default:
                break;
        }
    }

    private void handleRotatorState(RotateState current, Event event) {
        switch (event) {
            case ROTATOR_TOGGLE:
                // Handle toggle of left claw
                if (current == RotateState.FLAT) {
                    rotateState = RotateState.SCORE;
                }
                if (current == RotateState.SCORE) {
                    rotateState = RotateState.FLAT;
                }
            default:
                break;
        }
    }

    // Toggle state between PICKUP and SCORE (if not in INIT state)
    // Puts the sub systems in the next state so they can move to it
    // when loop is called
    public void toggleState() {
        handleIntakeState(intakeState, Event.INTAKE_TOGGLE);
    }

    public void toggleRotator() {
        handleRotatorState(rotateState, Event.ROTATOR_TOGGLE);
    }

    public void toggleLeftClaw() {
        handleLeftClawState(leftClawState, Event.LEFT_CLAW_TOGGLE);
    }

    public void toggleRightClaw() {
        handleRightClawState(rightClawState, Event.RIGHT_CLAW_TOGGLE);
    }

    private void moveArmToPosition(double armPosition) {
        armMotor.set(armPosition);
    }

    private void moveRotatorToPosition(double rotatePosition) {
        rotator.setPosition(rotatePosition);
    }

    private void moveClawToPosition(Servo claw, double clawPosition) {
        claw.setPosition(clawPosition);
    }

    public void loop() {
        // Try to get to the states needed
        switch(armState) {
            case SCORE:
                moveArmToPosition(ARM_SCORE_POSITION);
                break;
            case PICKUP:
                moveArmToPosition(ARM_PICKUP_POSITION);
                break;
            default:
                break;
        }

        switch(rotateState) {
            case SCORE:
                moveRotatorToPosition(ROTATOR_SCORE_POSITION);
                break;
            case FLAT:
                moveRotatorToPosition(ROTATOR_FLAT_POSITION);
                break;
            default:
                break;
        }

        switch (leftClawState) {
            case OPEN:
                moveClawToPosition(leftClaw, LEFT_CLAW_OPEN_POSITION);
                break;
            case CLOSED:
                moveClawToPosition(leftClaw, LEFT_CLAW_CLOSE_POSITION);
                break;
            default:
                break;
        }

        switch (rightClawState) {
            case OPEN:
                moveClawToPosition(rightClaw, RIGHT_CLAW_OPEN_POSITION);
                break;
            case CLOSED:
                moveClawToPosition(rightClaw, RIGHT_CLAW_CLOSE_POSITION);
                break;
            default:
                break;
        }
    }

}
