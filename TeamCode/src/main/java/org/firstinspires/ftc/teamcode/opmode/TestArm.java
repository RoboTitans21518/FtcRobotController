package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="TestArm", group="Linear OpMode")
public class TestArm extends LinearOpMode {
    private DcMotorEx arm;

    public static int upPosition = 175;
    public static int downPosition = 15;

    public static int curPosition = 0;

    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotorEx.class, "armMotor");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(upPosition);
        arm.setVelocity(1000);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                arm.setTargetPosition(upPosition);
                arm.setPower(1);
                arm.setVelocity(1000);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                telemetry.addLine("A button");
            }
            if (gamepad1.b) {
                arm.setTargetPosition(downPosition);
                arm.setPower(1);
                arm.setVelocity(1000);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                telemetry.addLine("B button");
            }

            TestArm.curPosition = arm.getCurrentPosition();
            double target = arm.getTargetPosition();

            telemetry.log().clear();
            telemetry.addData("Target:", target);
            telemetry.addData("Cur Position:", TestArm.curPosition);
            telemetry.update();
        }
    }
}
