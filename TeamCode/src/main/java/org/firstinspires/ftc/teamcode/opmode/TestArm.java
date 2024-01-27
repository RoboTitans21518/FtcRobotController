package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="TestArm", group="Linear OpMode")
public class TestArm extends LinearOpMode {
    private Motor arm;

    public static double power = 0;
    @Override
    public void runOpMode() {
        arm = new Motor(hardwareMap, "arm", Motor.GoBILDA.RPM_312);
        arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (!isStopRequested()) {
            arm.set(power);
        }
    }
}
