package org.whitneyrobotics.ftc.subsys;

import android.util.Range;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.whitneyrobotics.ftc.subsys.Lift;
import lib.subsys.robot.WHSRobot;
import lib.util.Coordinate;
import lib.util.Functions;
import lib.util.PIDController;
import lib.util.Position;
import lib.util.RobotConstants;
import lib.util.Toggler;

import static lib.util.Functions.cosd;
import static lib.util.Functions.positionArrayToDoubleArray;
import static lib.util.Functions.sind;

/**
 * Created by Jason on 10/20/2017.
 */

public class WHSRobotImpl implements WHSRobot {

    public Drivetrain drivetrain;
    public IMU imu;
    //public Intake intake;
    //public Lift lift;

    Coordinate currentCoord;
    private double targetHeading; //field frame
    public double angleToTargetDebug;
    public double distanceToTargetDebug = 0;
    private double lastKnownHeading = 0.1;

    private static double DEADBAND_DRIVE_TO_TARGET = RobotConstants.DEADBAND_DRIVE_TO_TARGET; //in mm
    private static double DEADBAND_ROTATE_TO_TARGET = RobotConstants.DEADBAND_ROTATE_TO_TARGET; //in degrees

    public static double DRIVE_MIN = RobotConstants.drive_min;
    public static double DRIVE_MAX = RobotConstants.drive_max;
    public static double ROTATE_MIN = RobotConstants.rotate_min;
    public static double ROTATE_MAX = RobotConstants.rotate_max;

    private static double ROTATE_KP = RobotConstants.R_KP;
    private static double ROTATE_KI = RobotConstants.R_KI;
    private static double ROTATE_KD = RobotConstants.R_KD;

    private static double DRIVE_KP = RobotConstants.D_KP;
    private static double DRIVE_KI = RobotConstants.D_KI;
    private static double DRIVE_KD = RobotConstants.D_KD;

    public PIDController rotateController = new PIDController(ROTATE_KP, ROTATE_KI, ROTATE_KD);
    public PIDController driveController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);

    private boolean firstRotateLoop = true;
    private boolean firstDriveLoop = true;
    private boolean driveBackwards;

    private int driveSwitch = 0;

    private boolean driveToTargetInProgress = false;
    private boolean rotateToTargetInProgress = false;

    private double [] encoderDeltas;
    private double robotX;
    private double robotY;
    private double distance;

    private static double MAXIMUM_ACCELERATION = 0;

    public double swerveDistanceToTargetDebug;
    public WHSRobotImpl(HardwareMap hardwareMap){
        DEADBAND_DRIVE_TO_TARGET = RobotConstants.DEADBAND_DRIVE_TO_TARGET; //in mm
        DEADBAND_ROTATE_TO_TARGET = RobotConstants.DEADBAND_ROTATE_TO_TARGET; //in degrees

        DRIVE_MIN = RobotConstants.drive_min;
        DRIVE_MAX = RobotConstants.drive_max;
        ROTATE_MIN = RobotConstants.rotate_min;
        ROTATE_MAX = RobotConstants.rotate_max;

        ROTATE_KP = RobotConstants.R_KP;
        ROTATE_KI = RobotConstants.R_KI;
        ROTATE_KD = RobotConstants.R_KD;

        DRIVE_KP = RobotConstants.D_KP;
        DRIVE_KI = RobotConstants.D_KI;
        DRIVE_KD = RobotConstants.D_KD;

        drivetrain = new Drivetrain(hardwareMap);
        imu = new IMU(hardwareMap);
        // lift = new Lift(hardwareMap);
        //intake = new Intake(hardwareMap);

        currentCoord = new Coordinate(0.0, 0.0, 150.0, 0.0);
    }

    @Override
    public void driveToTarget(Position targetPos, boolean backwards) {
        Position vectorToTarget = Functions.subtractPositions(targetPos, currentCoord.getPos()); //field frame
        vectorToTarget = field2body(vectorToTarget); //body frame
        double distanceToTarget = vectorToTarget.getX()/*Functions.calculateMagnitude(vectorToTarget) * (vectorToTarget.getX() >= 0 ? 1 : -1)*/;
        distanceToTargetDebug = distanceToTarget;

        double degreesToRotate = Math.atan2(vectorToTarget.getY(), vectorToTarget.getX()); //from -pi to pi rad
        degreesToRotate = degreesToRotate * 180 / Math.PI;
        targetHeading = Functions.normalizeAngle(currentCoord.getHeading() + degreesToRotate); //-180 to 180 deg

        switch (driveSwitch) {
            case 0:
                driveToTargetInProgress = true;
                rotateToTarget(targetHeading, backwards);
                if (!rotateToTargetInProgress()) {
                    driveSwitch = 1;
                }
                break;
            case 1:

                if (firstDriveLoop) {
                    driveToTargetInProgress = true;
                    driveController.init(distanceToTarget);
                    firstDriveLoop = false;
                }

                driveController.setConstants(DRIVE_KP, DRIVE_KI, DRIVE_KD);
                driveController.calculate(distanceToTarget);

                double power = Functions.map(Math.abs(driveController.getOutput()), DEADBAND_DRIVE_TO_TARGET, 1500, DRIVE_MIN, DRIVE_MAX);

                // this stuff may be causing the robot to oscillate around the target position
                if (distanceToTarget < 0) {
                    power = -power;
                } else if (distanceToTarget > 0){
                    power = Math.abs(power);
                }
                if (Math.abs(distanceToTarget) > DEADBAND_DRIVE_TO_TARGET) {
                    driveToTargetInProgress = true;
                    drivetrain.operateLeft(power);
                    drivetrain.operateRight(power);
                } else {
                    drivetrain.operateRight(0.0);
                    drivetrain.operateLeft(0.0);
                    driveToTargetInProgress = false;
                    rotateToTargetInProgress = false;
                    firstDriveLoop = true;
                    driveSwitch = 0;
                }
                // end of weird code
                break;
        }
    }

    @Override
    public void rotateToTarget(double targetHeading, boolean backwards) {

        double angleToTarget = targetHeading - currentCoord.getHeading();
        /*if (backwards && angleToTarget > 90) {
            angleToTarget = angleToTarget - 180;
            driveBackwards = true;
        }
        else if (backwards && angleToTarget < -90) {
            angleToTarget = angleToTarget + 180;
            driveBackwards = true;
        }*/
        if (backwards) {
            angleToTarget = Functions.normalizeAngle(angleToTarget + 180); //-180 to 180 deg
            driveBackwards = true;
        }
        else {
            angleToTarget = Functions.normalizeAngle(angleToTarget);
            driveBackwards = false;
        }

        angleToTargetDebug = angleToTarget;

        if (firstRotateLoop) {
            rotateToTargetInProgress = true;
            rotateController.init(angleToTarget);
            firstRotateLoop = false;
        }

        rotateController.setConstants(ROTATE_KP, ROTATE_KI, ROTATE_KD);
        rotateController.calculate(angleToTarget);

        double power = (rotateController.getOutput() >= 0 ? 1 : -1) * (Functions.map(Math.abs(rotateController.getOutput()),  0, 180, ROTATE_MIN, ROTATE_MAX));

        if (Math.abs(angleToTarget) > DEADBAND_ROTATE_TO_TARGET/* && rotateController.getDerivative() < 40*/) {
            drivetrain.operateLeft(-power);
            drivetrain.operateRight(power);
            rotateToTargetInProgress = true;
        }
        else {
            drivetrain.operateLeft(0.0);
            drivetrain.operateRight(0.0);
            rotateToTargetInProgress = false;
            firstRotateLoop = true;
        }
    }

    public void swerveToTarget(Position[] targetPositions, int numToInject, double weightSmooth, double tolerance, double velocityConstant){
        double targetDoubles[][] = positionArrayToDoubleArray(targetPositions)
        double[][] injectedPath =  inject(targetDoubles,numToInject);
        double[][] smoothedPath =  smoothPath(injectedPath,1-weightSmooth,weightSmooth, tolerance);
        double [] distanceAtPoint = distanceAtPoint(smoothedPath);
        double []  curvatureAtPoint = calculateCurvature(smoothedPath);
    }

    double [] calculateTargetVelocities(double[][] smoothedPath, double k ){
        double[] targetVelocities = new double[smoothedPath.length];
        double a = MAXIMUM_ACCELERATION;
        for (int i = smoothedPath.length-1; i>=0; i--){
            double distance = Math.hypot(Math.abs(smoothedPath[i][0] - smoothedPath[i+1][0]), Math.abs(smoothedPath[i+1][1]-smoothedPath[i][1]));
            double targetVelocity = Math.min(k/calculateCurvature(smoothedPath)[i], Math.sqrt(targetVelocities[i+1] + 2 *a* distance))
        }
    }
    double[] calculateCurvature(double[][]smoothedPath){
        double [] curvatureArray = new double[smoothedPath.length];
        curvatureArray[0] = 0;
        curvatureArray[smoothedPath.length] =0;
        for(int i = 1; i<smoothedPath.length ; i++){
            double x1 = smoothedPath[i][0] + 0.0001;
            double y1 = smoothedPath [i][1];

            double x2 = smoothedPath[i-1][0];
            double y2 = smoothedPath [i-1][1];

            double x3 = smoothedPath[i+1][0];
            double y3 = smoothedPath [i+1][1];

            double k1 = 0.5*(Math.pow(x1,2) + Math.pow(y1,2)-Math.pow(x2,2)-Math.pow(y2,2))/(x1-x2);
            double k2 = (y1-y2)/(x1-x2);

            double b = 0.5*(Math.pow(x2,2)-2*x2*k1+Math.pow(y2,2)-Math.pow(x3,2)+2*x3*k1-Math.pow(y3,2))/(x3*k2-y3+y2-x2*k2);
            double a = k1-k2*b;

            Double r = new Double(Math.sqrt(Math.pow(x1-a,2) + (Math.pow(y1-b,2))));
            if (r.isNaN()){
                r = 0.0;
            }
            double curvature = 1/r;

            curvatureArray[i] = curvature;
        }
     return curvatureArray;
    }

    double [] distanceAtPoint(double[][] smoothPath){
        double[] distanceArray = new double[smoothPath.length];
        distanceArray[0] = 0;
        for (int i = 1; i<=smoothPath.length; i++){
            distanceArray[i] = distanceArray[i-1] + Math.hypot(Math.abs(smoothPath[i][0] - smoothPath[i-1][0]), Math.abs(smoothPath[i-1][1]-smoothPath[i][1]));
        }
        return distanceArray;
    }

    public double[][] smoothPath(double[][] path, double weight_data, double weight_smooth, double tolerance)
    {

        //copy array
        double[][] newPath = doubleArrayCopy(path);

        double change = tolerance;
        while(change >= tolerance)
        {
            change = 0.0;
            for(int i=1; i<path.length-1; i++)
                for(int j=0; j<path[i].length; j++)
                {
                    double aux = newPath[i][j];
                    newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
        }

        return newPath;

    }
    public static double[][] doubleArrayCopy(double[][] arr)
    {

        //size first dimension of array
        double[][] temp = new double[arr.length][arr[0].length];

        for(int i=0; i<arr.length; i++)
        {
            //Resize second dimension of array
            temp[i] = new double[arr[i].length];

            //Copy Contents
            for(int j=0; j<arr[i].length; j++)
                temp[i][j] = arr[i][j];
        }

        return temp;

    }
    private double[][] inject(double[][] orig, int numToInject) {
        double morePoints[][];

        //create extended 2 Dimensional array to hold additional points
        morePoints = new double[orig.length + ((numToInject)*(orig.length-1))][2];

        int index=0;

        //loop through original array
        for(int i=0; i<orig.length-1; i++)
        {
            //copy first
            morePoints[index][0] = orig[i][0];
            morePoints[index][1] = orig[i][1];
            index++;

            for(int j=1; j<numToInject+1; j++)
            {
                //calculate intermediate x points between j and j+1 original points
                morePoints[index][0] = j*((orig[i+1][0]-orig[i][0])/(numToInject+1))+orig[i][0];

                //calculate intermediate y points  between j and j+1 original points
                morePoints[index][1] = j*((orig[i+1][1]-orig[i][1])/(numToInject+1))+orig[i][1];

                index++;
            }
        }

        //copy last
        morePoints[index][0] =orig[orig.length-1][0];
        morePoints[index][1] =orig[orig.length-1][1];
        index++;

        return morePoints;
    }

    @Override
    public boolean driveToTargetInProgress() {
        return driveToTargetInProgress;
    }

    @Override
    public boolean rotateToTargetInProgress() {
        return rotateToTargetInProgress;
    }

    @Override
    public void estimatePosition() {
        encoderDeltas = drivetrain.getEncoderDelta();
        distance = drivetrain.encToMM((encoderDeltas[0] + encoderDeltas[1])/2);
        robotX += distance*Math.cos(Math.toRadians(getCoordinate().getHeading()));
        robotY += distance*Math.sin(Math.toRadians(getCoordinate().getHeading()));
        currentCoord.setX(robotX);
        currentCoord.setY(robotY);
    }
    @Override
    public void estimateHeading() {
        double currentHeading;
        currentHeading = Functions.normalizeAngle(imu.getHeading() + imu.getImuBias()); //-180 to 180 deg
        if (currentHeading != 0.0) {
            lastKnownHeading = currentHeading;
        }
        currentCoord.setHeading(lastKnownHeading); //updates global variable
    }

    @Override
    public void setInitialCoordinate(Coordinate initCoord) {
        currentCoord = initCoord;
        imu.setImuBias(currentCoord.getHeading());
    }

    @Override
    public void setCoordinate(Coordinate coord) {
        currentCoord = coord;
        imu.setImuBias(currentCoord.getHeading());
    }

    @Override
    public Coordinate getCoordinate() {
        return currentCoord;
    }

    public Position body2field(Position bodyVector)
    {
        Position fieldVector;
        double heading = currentCoord.getHeading();

        double[][] C_b2f = {{cosd(heading),  -Functions.sind(heading),  0},
                {Functions.sind(heading),   cosd(heading),  0},
                {0,                         0,                        1}};

        fieldVector = Functions.transformCoordinates(C_b2f,bodyVector);
        return fieldVector;

    }

    public Position field2body(Position fieldVector)
    {
        Position bodyVector;
        double heading = currentCoord.getHeading();

        double[][] C_f2b = {{ cosd(heading),   Functions.sind(heading),  0},
                {-Functions.sind(heading),   cosd(heading),  0},
                { 0,                         0,                        1}};

        bodyVector = Functions.transformCoordinates(C_f2b,fieldVector);
        return bodyVector;

    }

    public Position front2back(Position frontVector)
    {
        Position backVector;
        double heading = 180;

        double[][] C_f2b = {{ -1,  0, 0},
                {  0, -1, 0},
                {  0,  0, 1}};

        backVector = Functions.transformCoordinates(C_f2b,frontVector);
        return backVector;
    }

}
