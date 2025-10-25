package org.firstinspires.ftc.teamcode.swerveDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TestClass extends OpMode {

    AxonAnalog flEncoder;
    AxonAnalog blEncoder;
    AxonAnalog brEncoder;
    AxonAnalog frEncoder;

    double realMaxVolts = 3.1865;

    @Override
    public void init(){
        CRServo flServo = hardwareMap.get(CRServo.class, "fl_rotation");
        CRServo blServo = hardwareMap.get(CRServo.class, "bl_rotation");
        CRServo brServo = hardwareMap.get(CRServo.class, "br_rotation");
        CRServo frServo = hardwareMap.get(CRServo.class, "fr_rotation");
        flEncoder = new AxonAnalog(hardwareMap, "fl_absolute", 2.885, realMaxVolts, true);
        frEncoder = new AxonAnalog(hardwareMap, "fr_absolute", 1.344, realMaxVolts, true);
        blEncoder = new AxonAnalog(hardwareMap, "bl_absolute", 4.707, realMaxVolts, true);
        brEncoder = new AxonAnalog(hardwareMap, "br_absolute", 5.153, realMaxVolts, true);

        flServo.setPower(0);
        blServo.setPower(0);
        brServo.setPower(0);
        frServo.setPower(0);
    }

    @Override
    public void loop(){
        telemetry.addData("FLServo", flEncoder.getHeading());
        telemetry.addData("BLServo", blEncoder.getHeading());
        telemetry.addData("BRServo", brEncoder.getHeading());
        telemetry.addData("FRServo", frEncoder.getHeading());

        telemetry.update();
    }
}
