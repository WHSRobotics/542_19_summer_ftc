package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.subsys.Drivetrain;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

@TeleOp(name = "Mecanum Drive Test")
public class MecanumDriveTest extends OpMode{
    WHSRobotImpl robot;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
    }

    @Override
    public void loop() {
        robot.estimateHeading();
        telemetry.addData("Current Heading", robot.getCoordinate().getHeading());
            if (gamepad1.left_bumper){
            robot.operateMecanumDrive(gamepad1.left_stick_x/2.54, gamepad1.left_stick_y/2.54, gamepad1.right_stick_x/2.54);
        }else{
            robot.operateMecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

    }
}
