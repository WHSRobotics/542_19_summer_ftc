package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

import lib.util.Coordinate;

@TeleOp (name = "Estimate Position Test")
public class estimatePositionTest extends OpMode {

    WHSRobotImpl robot;
    double maxAccel = 0;
    double currentAccel;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.setInitialCoordinate(new Coordinate(0, 0, 0, 90));
    }

    @Override
    public void loop() {

        robot.drivetrain.operate(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        currentAccel = Math.abs(robot.imu.getYAcceleration());
        robot.estimateHeading();
        robot.estimatePosition();
        if (currentAccel > maxAccel){
            maxAccel = currentAccel;
        }
        telemetry.addData("Max Y Accel",maxAccel);
        /*telemetry.addData("x", robot.getCoordinate().getX());
        telemetry.addData("y", robot.getCoordinate().getY());
        telemetry.addData("IMU", robot.imu.getHeading());
        telemetry.addData("X", robot.getCoordinate().getX());
        telemetry.addData("Y", robot.getCoordinate().getY());
        telemetry.addData("Z", robot.getCoordinate().getZ());
        telemetry.addData("Heading", robot.getCoordinate().getHeading());
        telemetry.addData("FL Position", robot.drivetrain.frontLeft.getCurrentPosition());
        telemetry.addData("BL Position", robot.drivetrain.backLeft.getCurrentPosition());
        telemetry.addData("FR Position", robot.drivetrain.frontRight.getCurrentPosition());
        telemetry.addData("BR Position", robot.drivetrain.backRight.getCurrentPosition());
        telemetry.addData("X heading: ", robot.imu.getThreeHeading()[0]);
        telemetry.addData("Y heading: ", robot.imu.getThreeHeading()[1]);
        telemetry.addData("Z heading: ", robot.imu.getThreeHeading()[2]);*/

    }
}