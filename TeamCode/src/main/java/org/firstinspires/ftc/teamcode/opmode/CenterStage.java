package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name="CenterStage", group="Linear OpMode")
public class CenterStage extends LinearOpMode {
    private Robot robot;
    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, gamepad1);

        waitForStart();

        while (!isStopRequested()) {
            robot.loop();
        }
    }
}
