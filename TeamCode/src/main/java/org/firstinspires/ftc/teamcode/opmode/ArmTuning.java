package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/*
 * Use velocity instead of power. More control.
 * - Do a MAX velocity for the motor. Set the maxVelocity to 80%. (2580 for 312 RPN motor.
 */
@Config
@TeleOp(name="ArmTuning", group="Linear OpMode")
public class ArmTuning extends OpMode {
    private PIDController controller;

    public static double MAX_VELOCITY_GOBUILDA_312 = 2580;
    public static double TICKS_PER_REV_GOBUILDA_312 = 537;

    // F value is ~ 32767/(MAX_VELOCITY)
    public static double f = 7.5;

    // P = 0.1 * F, I = 0.1 *P, D=0, position value to 5.
    public static double p = 0, i = 0, d = 0;

    public static int target = 0;

    public static double targetVelocity = 10;

    private final double ticks_in_degree = TICKS_PER_REV_GOBUILDA_312/360;

    private DcMotorEx armMotor;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                FtcDashboard.getInstance().getTelemetry()
        );
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        loop1();
    }

    public void loop1() {
        controller.setPID(p, i, d);
        double armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double radians = Math.toRadians(target/ticks_in_degree);
        double ff = Math.cos(radians) * f;
        double power = pid + ff;
        power = Range.clip(power, -0.4, 0.4);
        armMotor.setPower(power);

        telemetry.addData("Arm Pos:", armPos);
        telemetry.addData("Target:", target);
        telemetry.addData("Power:", power);
        telemetry.addData("Radians:", radians);
        telemetry.addData("FF:", ff);
        telemetry.update();
    }

    private void loop2() {
        controller.setPIDF(p, i, d, f);
        double curPosition = armMotor.getCurrentPosition();
        double curVelocity = armMotor.getVelocity();
        double velocityPID = controller.calculate(curVelocity, targetVelocity);
        double radians = Math.toRadians(curPosition / ticks_in_degree);
        double ff = Math.cos(radians) * f;
        double velocity = velocityPID + ff;

        armMotor.setVelocityPIDFCoefficients(p, i, d, f);
        armMotor.setVelocity(velocity);

        telemetry.log().clear();
        telemetry.addData("Position:", curPosition);
        telemetry.addData("Velocity:", curVelocity);
        telemetry.addData("Calc Velocity:", velocity);
        telemetry.addData("Velocity PID:", velocityPID);
        telemetry.addData("Radians:", radians);
        telemetry.addData("Cos:", Math.cos(radians));
        telemetry.addData("FF:", ff);
        telemetry.update();
    }
    /*
    @Override
    public void runOpMode() {
        controller = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setVelocityPIDFCoefficients(p, i, d, f);
        armMotor.setPositionPIDFCoefficients(5);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            controller.setPIDF(p, i, d, f);
            double curPosition = armMotor.getCurrentPosition();
            double curVelocity = armMotor.getVelocity();
            double velocityPID = controller.calculate(curVelocity, targetVelocity);
            double radians = Math.toRadians(curPosition / ticks_in_degree);
            double ff = Math.cos(radians) * f;
            double velocity = velocityPID + ff;

            armMotor.setVelocityPIDFCoefficients(p, i, d, f);
            armMotor.setVelocity(velocity);

            telemetry.log().clear();
            telemetry.addData("Position:", curPosition);
            telemetry.addData("Velocity:", curVelocity);
            telemetry.addData("Calc Velocity:", velocity);
            telemetry.addData("Velocity PID:", velocityPID);
            telemetry.addData("Radians:", radians);
            telemetry.addData("Cos:", Math.cos(radians));
            telemetry.addData("FF:", ff);
            telemetry.update();
        }
    }

     */
}

