package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="TestArm", group="Linear OpMode")
public class TestArm extends LinearOpMode {
    private DcMotorEx arm;

    public static double MAX_VELOCITY = 0;
    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotorEx.class, "test");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (!isStopRequested()) {
            arm.setPower(1);
            double curVelocity = arm.getVelocity();

            if (curVelocity > MAX_VELOCITY) MAX_VELOCITY = curVelocity;

            telemetry.log().clear();
            telemetry.addData("Current Velocity:", curVelocity);
            telemetry.addData("Max Velocity:", MAX_VELOCITY);
            telemetry.update();
        }
    }
}
