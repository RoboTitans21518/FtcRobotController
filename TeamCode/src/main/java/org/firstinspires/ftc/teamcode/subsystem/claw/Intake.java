package org.firstinspires.ftc.teamcode.subsystem.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Intake {
    private Servo leftClaw;
    private Servo rightClaw;

    public void init(HardwareMap hwMap) {
        leftClaw = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");
    }

    
}
