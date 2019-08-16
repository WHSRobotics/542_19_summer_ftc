package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Eileen's OpMode", group = "a")
public class EileensOpMode extends OpMode {

    DcMotorEx motor1;


    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, "driveFL");
    }

    @Override
    public void loop() {
        motor1.setPower(1.0);
        telemetry.addData("Velocity", motor1.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("Motor Encoder", motor1.getCurrentPosition());
    }
}