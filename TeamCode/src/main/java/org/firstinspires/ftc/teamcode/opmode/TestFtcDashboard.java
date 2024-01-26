package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="TestFtcDashboard", group="Linear OpMode")
public class TestFtcDashboard extends LinearOpMode {
    public static boolean OPEN = false;
    private boolean prevState = false;

    FtcDashboard dashboard;

    private Servo leftClaw;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        TelemetryPacket packet = new TelemetryPacket();
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        waitForStart();

        while (!isStopRequested()) {
            if (OPEN == prevState) continue;

            if (OPEN) {
                leftClaw.setPosition(0.1);
                packet.addLine("CLAW OPEN");
                packet.put("claw", "open");
                dashboard.sendTelemetryPacket(packet);
            } else {
                leftClaw.setPosition(0.7);
                packet.addLine("CLAW CLOSE");
                packet.put("claw", "close");
                dashboard.sendTelemetryPacket(packet);
            }

            prevState = OPEN;
        }
    }
}
