package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

import lib.util.Coordinate;
import lib.util.Position;
@Autonomous(name = "Swerve to target test" )
public class SwerveToTargetTest extends OpMode {
    WHSRobotImpl robot;
    Coordinate startingCoordinate = new Coordinate(0,0,0,0);
    Position p1 = new Position(600,600,150);

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.setInitialCoordinate(startingCoordinate);
    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();

        robot.swerveToTarget(p1,.25,0,.15);
        telemetry.addData("Distance To Target", robot.swerveDistanceToTargetDebug);
        telemetry.addData("x", robot.getCoordinate().getX());
        telemetry.addData("y", robot.getCoordinate().getY());
        telemetry.addData("heading", robot.getCoordinate().getHeading());
    }
}
