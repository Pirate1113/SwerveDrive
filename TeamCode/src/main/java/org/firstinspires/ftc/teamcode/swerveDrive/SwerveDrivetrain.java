package org.firstinspires.ftc.teamcode.swerveDrive;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class SwerveDrivetrain implements Subsystem {
    public static final SwerveDrivetrain INSTANCE = new SwerveDrivetrain();
    private SwerveDrivetrain() {}

    private final double ANALOG_VOLTAGE_COMPENSATION = 3.22;
    public SwerveModule fl_Module, bl_Module, br_Module, fr_Module;

    private final double
            fl_offset = 3.03,
            bl_offset = 4.62,
            br_offset = 1.422,
            fr_offset = 1.3328;

    public SwerveModule[] swerveModules;

    @Override
    public void initialize(){
        fl_Module = new SwerveModule(new MotorEx("fl_motor"), "fl_rotation", false,
                "fl_absolute", fl_offset, false, ANALOG_VOLTAGE_COMPENSATION);

        bl_Module = new SwerveModule(new MotorEx("bl_motor").reversed(), "bl_rotation", false,
                "bl_absolute", bl_offset, false, ANALOG_VOLTAGE_COMPENSATION);

        br_Module = new SwerveModule(new MotorEx("br_motor").reversed(), "br_rotation", false,
                "br_absolute", br_offset, false, ANALOG_VOLTAGE_COMPENSATION);

        fr_Module = new SwerveModule(new MotorEx("fr_motor"), "fr_rotation", false,
                "fr_absolute", fr_offset, false, ANALOG_VOLTAGE_COMPENSATION);

        swerveModules = new SwerveModule[]{fl_Module, bl_Module, br_Module, fr_Module};
    }

    @Override
    public void periodic(){
        double rawLeftX = -ActiveOpMode.gamepad1().left_stick_x,
                rawLeftY = ActiveOpMode.gamepad1().left_stick_y,
                rawRightX = -ActiveOpMode.gamepad1().right_stick_x;

        double[][] vectorPositions = {
                {-1, 1}, //FL
                {-1, -1}, //BL
                {1, -1}, //BR
                {1, 1} //FR
        };
        double[] wheelSpeeds = new double[swerveModules.length];
        double[] rotationAngles = new double[swerveModules.length];

        for (int i = 0; i < swerveModules.length; i++) {
            double vectorX = vectorPositions[i][0];
            double vectorY = vectorPositions[i][1];

            // Compute rotation vector (perpendicular to wheel position)
            double rotX = vectorY * rawRightX;
            double rotY =  -vectorX * rawRightX;

            // Combine translation + rotation
            double resultX = rawLeftX + rotX;
            double resultY = rawLeftY + rotY;

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

        // === Apply to each module ===
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].rotateTo(rotationAngles[i]);
            swerveModules[i].setMotorPower(wheelSpeeds[i]);
        }

    }
}
