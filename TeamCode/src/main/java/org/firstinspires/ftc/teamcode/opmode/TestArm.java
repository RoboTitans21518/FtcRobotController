package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="TestArm", group="Linear OpMode")
public class TestArm extends LinearOpMode {
    private Motor arm;

    public static int targetPosition = 1200;

    public static double kP = 0.05;
    public static double kV = 0.1;

    @Override
    public void runOpMode() {
        PIDFController armController = new PIDFController(kP, 0, 0, kV);
        arm = new Motor(hardwareMap, "armMotor", Motor.GoBILDA.RPM_312);
        arm.setInverted(true);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (!isStopRequested()) {
            double currentPosition = arm.getCurrentPosition();
            double position = armController.calculate(currentPosition);
            arm.motor.setPower(position);

            telemetry.log().clear();
            telemetry.addData("Current Velocity:", targetPosition);
            telemetry.addData("Power:", position);
            telemetry.addData("Max Velocity:", arm.getCurrentPosition());
            telemetry.update();
        }
    }
}
