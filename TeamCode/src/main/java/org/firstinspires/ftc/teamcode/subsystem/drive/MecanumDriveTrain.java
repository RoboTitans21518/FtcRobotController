package org.firstinspires.ftc.teamcode.subsystem.drive;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

public class MecanumDriveTrain {
    private MecanumDrive drive;
    private boolean active;
    double leftX;
    double leftY;
    double rightX;

    public MecanumDriveTrain(HardwareMap hwMap) {
        Motor frontL = new Motor(hwMap, "frontL", Motor.GoBILDA.RPM_312);
        Motor frontR = new Motor(hwMap, "frontR", Motor.GoBILDA.RPM_312);
        Motor backL = new Motor(hwMap, "backL", Motor.GoBILDA.RPM_312);
        Motor backR = new Motor(hwMap, "backR", Motor.GoBILDA.RPM_312);
        frontR.setInverted(true);
        backR.setInverted(true);
        drive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(frontL, frontR, backL, backR);
        drive.stop();
        active = false;
        leftX = 0.0;
        leftY = 0.0;
        rightX = 0.0;
    }

    public void updateInputs(double leftX, double leftY, double rightX) {
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        active = true;
    }

    public void loop() {
        if (!active) return;

        drive.driveRobotCentric(
                leftX,
                leftY,
                rightX,
                false
        );
    }
}
