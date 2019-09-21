package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Eileen's OpMode", group = "a")
public class EileensOpMode extends OpMode {

    DcMotor motor1;


    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor");
    }

    @Override
    public void loop() {
        motor1.setPower(1.0);
        telemetry.addData("Motor Encoder", motor1.getCurrentPosition());
    }
}