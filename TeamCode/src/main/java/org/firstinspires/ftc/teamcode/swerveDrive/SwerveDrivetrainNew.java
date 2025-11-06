package org.firstinspires.ftc.teamcode.swerveDrive;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class SwerveDrivetrainNew implements Subsystem {
    public static final SwerveDrivetrainNew INSTANCE = new SwerveDrivetrainNew();
    private SwerveDrivetrainNew() {}

    private final double ANALOG_VOLTAGE_COMPENSATION = 3.1865;
    public SwerveModule fl_Module, bl_Module, br_Module, fr_Module;
    public SwerveModule[] swerveModules;

    @Override
    public void initialize(){
        fl_Module = new SwerveModule(new MotorEx("fl_motor").reversed(), "fl_rotation", true,
                "fl_absolute", 6.046, true, ANALOG_VOLTAGE_COMPENSATION, -1, 1);

        bl_Module = new SwerveModule(new MotorEx("bl_motor"), "bl_rotation", true,
                "bl_absolute", 4.695, true, ANALOG_VOLTAGE_COMPENSATION, -1, -1);

        br_Module = new SwerveModule(new MotorEx("br_motor"), "br_rotation", true,
                "br_absolute", 2.007, true, ANALOG_VOLTAGE_COMPENSATION, 1, -1);

        fr_Module = new SwerveModule(new MotorEx("fr_motor").reversed(), "fr_rotation", true,
                "fr_absolute", 1.351, true, ANALOG_VOLTAGE_COMPENSATION, 1, 1);

        swerveModules = new SwerveModule[]{fl_Module, bl_Module, br_Module, fr_Module};
    }

    @Override
    public void periodic(){
        double rawLeftX = ActiveOpMode.gamepad1().left_stick_x,
                rawLeftY = -ActiveOpMode.gamepad1().left_stick_y,
                rawRightX = ActiveOpMode.gamepad1().right_stick_x,
                realRightX = rawRightX / Math.sqrt(2);

        //double flPivotAngle = Math.PI/4, frPivotAngle = -Math.PI/4,
              //  brPivotAngle = -3*Math.PI/4, blPivotAngle = 3*Math.PI/4;
        //double[] pivotAngles = {flPivotAngle, blPivotAngle, brPivotAngle, frPivotAngle};

        double[] wheelSpeeds = new double[swerveModules.length];
        double[] rotationAngles = new double[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++) {
            double rotVectorX = realRightX * swerveModules[i].yOffset;
            double rotVectorY = -1 * realRightX * swerveModules[i].xOffset;

            double resultX = rawLeftX + rotVectorX;
            double resultY = rawLeftY + rotVectorY;

            // Compute final speed + angle
            wheelSpeeds[i] = Math.sqrt(resultX * resultX + resultY * resultY);
            rotationAngles[i] = Math.atan2(resultY, resultX);
        }

        // === Normalize wheel speeds so none exceed 1.0 ===
        double max = Math.max(Math.max(wheelSpeeds[0], wheelSpeeds[1]),
                Math.max(wheelSpeeds[2], wheelSpeeds[3]));
        if (max > 1.0) {
            for (int i = 0; i < swerveModules.length; i++)
            {wheelSpeeds[i] /= max;}
        }

        for (int i = 0; i < swerveModules.length; i++){
            ActiveOpMode.telemetry().addData("target", rotationAngles[i]); //remove later
            ActiveOpMode.telemetry().addData("heading", swerveModules[i].getPodHeading()-Math.PI); //remove later
        }

        // === Apply to each module ===
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].rotateTo(rotationAngles[i]);
            swerveModules[i].setMotorPower(wheelSpeeds[i]);
        }

    }
}
