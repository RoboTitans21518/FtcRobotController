package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

public class MecanumDriveTrain {
    private MecanumDrive drive;
    double leftX;
    double leftY;
    double rightX;

    public MecanumDriveTrain(HardwareMap hwMap) {
        drive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(
                new Motor(hwMap, "frontL", Motor.GoBILDA.RPM_312),
                new Motor(hwMap, "frontR",  Motor.GoBILDA.RPM_312),
                new Motor(hwMap, "backL",  Motor.GoBILDA.RPM_312),
                new Motor(hwMap, "backR",  Motor.GoBILDA.RPM_312)
        );
        leftX = 0.0;
        leftY = 0.0;
        rightX = 0.0;
    }

    public void updateInputs(double leftX, double leftY, double rightX) {
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
    }

    public void loop() {
        drive.driveRobotCentric(
                -leftX,
                -leftY,
                -rightX,
                false
        );
    }

}
