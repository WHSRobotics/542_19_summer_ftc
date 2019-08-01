package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

import lib.subsys.robot.WHSRobot;
@TeleOp (name = "Estimate Position Test")
public class EstimatePositionTest extends OpMode {

    WHSRobot robot;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
    }

    @Override
    public void loop() {

        robot.drivetrain().operate(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

        robot.estimateHeading();
        robot.estimatePosition();
        telemetry.addData("x", robot.getCoordinate().getX());
        telemetry.addData("y", robot.getCoordinate().getY());
        telemetry.addData("heading", robot.getCoordinate().getHeading());

    }
}
