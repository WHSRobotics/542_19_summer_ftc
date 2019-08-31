package org.whitneyrobotics.ftc.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

import lib.util.Coordinate;
import lib.util.Position;
import lib.util.RobotConstants;
import lib.util.SwerveToTarget;

@Autonomous(name = "Swerve to target test" )
public class SwerveToTargetTest extends OpMode {
    WHSRobotImpl robot;
    Coordinate startingCoordinate = new Coordinate(0,0,0,90);
    Position p1 = new Position(0, 3000, 150);
    Position p2 = new Position(600,600,150);
    Position[] positions = {startingCoordinate.getPos(), p1};
    SwerveToTarget swerve1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    double kp = RobotConstants.S_KP;
    double kv = RobotConstants.S_KV;
    double ka = RobotConstants.S_KA;

    @Override
    public void init() {
        TelemetryPacket packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new WHSRobotImpl(hardwareMap);
        robot.setInitialCoordinate(startingCoordinate);
        swerve1 = new SwerveToTarget(kp, kv ,ka, positions, 245, 0.8542, 0.001, 2, 300, robot.drivetrain.getTrackWidth());
        telemetry.setMsTransmissionInterval(10);

    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();

        double[] swerve1Powers = swerve1.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());
        robot.drivetrain.operate(swerve1Powers[0], swerve1Powers[1]);
        telemetry.addData("target left wheel velocity", swerve1.getCurrentTargetWheelVelocities()[0]);
        telemetry.addData("target right wheel velocity", swerve1.getCurrentTargetWheelVelocities()[1]);
        telemetry.addData("actual left wheel velocity", robot.drivetrain.getWheelVelocities()[0]);
        telemetry.addData("actual right wheel velocity", robot.drivetrain.getWheelVelocities()[1]);
        telemetry.addData("x", robot.getCoordinate().getX());
        telemetry.addData("y", robot.getCoordinate().getY());
        telemetry.addData("heading", robot.getCoordinate().getHeading());
        telemetry.addData("index of last closest point", swerve1.lastClosestPointIndex);
        telemetry.addData("left power", swerve1Powers[0]);
        telemetry.addData("right power", swerve1Powers[1]);
        for (int i = 0; i < swerve1.smoothedPath.length; i++) {
            telemetry.addData("point" + i, swerve1.smoothedPath[i][0] + ", " + swerve1.smoothedPath[i][1]);
        }
        for (int i = 0; i < swerve1.targetVelocities.length; i++) {
            telemetry.addData("velocity" + i, swerve1.targetVelocities[i]);
        }
        telemetry.addData("lookahead point",  swerve1.lookaheadPoint[0] + ", " + swerve1.lookaheadPoint[1]);
    }
}
