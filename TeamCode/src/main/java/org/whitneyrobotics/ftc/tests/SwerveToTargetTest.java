package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

import lib.util.Coordinate;
import lib.util.Position;
import lib.util.SwerveToTarget;

@Autonomous(name = "Swerve to target test" )
public class SwerveToTargetTest extends OpMode {
    WHSRobotImpl robot;
    Coordinate startingCoordinate = new Coordinate(0,0,0,0);
    Position p1 = new Position(0, 300, 150);
    Position p2 = new Position(600,600,150);
    Position[] positions = {startingCoordinate.getPos(), p1, p2};
    SwerveToTarget swerve1;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.setInitialCoordinate(startingCoordinate);
        swerve1 = new SwerveToTarget(0.005, positions, 10, 0.8542, 0.001, 3, 80, robot.drivetrain.getTrackWidth());
    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();

        double[] swerve1Powers = swerve1.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());
        robot.drivetrain.operate(swerve1Powers[0], swerve1Powers[1]);

        telemetry.addData("x", robot.getCoordinate().getX());
        telemetry.addData("y", robot.getCoordinate().getY());
        telemetry.addData("heading", robot.getCoordinate().getHeading());
        telemetry.addData("index of last closest point", swerve1.lastClosestPointIndex);
        telemetry.addData("left power", swerve1Powers[0]);
        telemetry.addData("right power", swerve1Powers[1]);
        for (int i = 0; i < swerve1.smoothedPath.length; i++) {
            telemetry.addData("point" + i, swerve1.smoothedPath[i][0] + ", " + swerve1.smoothedPath[i][1]);
        }
    }
}
