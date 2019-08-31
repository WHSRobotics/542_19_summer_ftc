package lib.util;

public class SwerveToTarget {

    Coordinate currentCoord;
    double lookaheadDistance;
    double trackWidth;

    private static double MAXIMUM_ACCELERATION = 2000.0; // mm/s^2
    public int lastClosestPointIndex;
    private int lastIndex = 0;
    private double currentTValue = 0;

    double[][] targetDoubles;
    double[][] injectedPath;
    public double[][] smoothedPath;
    double[] distances;
    double[] targetCurvatures;

    public double[] targetVelocities;

    public double lastRateLimiterOutput = 0.0;
    private double[] lastTargetWheelVelocities = {0.0, 0.0};
    double lastTime;
    double kP;
    double kV;
    double kA;

    public double[] lookaheadPoint;

    private double[]currentTargetWheelVelocities;

    public SwerveToTarget(double kP, double kV, double kA, Position[] targetPositions, int numToInject, double weightSmooth, double tolerance, double velocityConstant, double lookaheadDistance, double trackWidth) {
        this.kP = kP;
        this.kV = kV;
        this.kA = kA;
        this.lookaheadDistance = lookaheadDistance;
        this.trackWidth = trackWidth;
        targetDoubles = Functions.positionArrayToDoubleArray(targetPositions);
        injectedPath = inject(targetDoubles, numToInject);
        smoothedPath = smoothPath(injectedPath, 1 - weightSmooth, weightSmooth, tolerance);
        distances = calculateDistanceAtPoint(smoothedPath);
        targetCurvatures = calculateTargetCurvatures(smoothedPath);
        targetVelocities = calculateTargetVelocities(smoothedPath, velocityConstant);
        lastTime = System.nanoTime() / 1E9;
        lastClosestPointIndex = 0;
    }

    public double[] calculateMotorPowers(Coordinate currentCoord, double[] currentWheelVelocities) {
        this.currentCoord = currentCoord;

        boolean tFound = false;
        for (int i = lastIndex; i < smoothedPath.length - 1; i++) {
            double[] startPoint = {smoothedPath[i][0], smoothedPath[i][1]};
            double[] endPoint = {smoothedPath[i + 1][0], smoothedPath[i + 1][1]};
            double nextTValue = calculateT(startPoint, endPoint, lookaheadDistance);

            if (!tFound && nextTValue != Double.NaN && (nextTValue + i) > (currentTValue + lastIndex)) {
                tFound = true;
                currentTValue = nextTValue;
                lastIndex = i;
            }
        }

        double[] calculatedTStartPoint = {smoothedPath[lastIndex][0], smoothedPath[lastIndex][1]};
        double[] calculatedTEndPoint = {smoothedPath[lastIndex + 1][0], smoothedPath[lastIndex + 1][1]};
        lookaheadPoint = Functions.Vectors.add(calculatedTStartPoint, Functions.Vectors.scale(currentTValue, Functions.Vectors.subtract(calculatedTEndPoint, calculatedTStartPoint)));

        double curvature = calculateCurvature(lookaheadDistance, lookaheadPoint);
        double targetVelocityAtClosestPoint = targetVelocities[calculateIndexOfClosestPoint(smoothedPath)];

        currentTargetWheelVelocities = calculateTargetWheelVelocities(targetVelocityAtClosestPoint, curvature);
        double deltaTime = System.nanoTime()/1E9 - lastTime;
        double[] targetWheelAccelerations = {(currentTargetWheelVelocities[0]-lastTargetWheelVelocities[0]) / deltaTime, (currentTargetWheelVelocities[1]-lastTargetWheelVelocities[1]) / deltaTime};

        double[] motorPowers = {0.0, 0.0};
        if (calculateIndexOfClosestPoint(smoothedPath) != smoothedPath.length - 1) {
            double[] feedForward = Functions.Vectors.add(Functions.Vectors.scale(kV, currentTargetWheelVelocities), Functions.Vectors.scale(kA, targetWheelAccelerations));
            double[] feedBack = Functions.Vectors.scale(kP, Functions.Vectors.subtract(currentTargetWheelVelocities, currentWheelVelocities));
            double[] calculatedMotorPowers = {Functions.constrain(feedForward[0] + feedBack[0], -1,1), Functions.constrain(feedForward[1] + feedBack[1],-1,1)};
            motorPowers = calculatedMotorPowers;
        }
        lastTargetWheelVelocities = currentTargetWheelVelocities;
        return motorPowers;
    }

    private double[][] inject(double[][] orig, int numToInject) {
        //create extended 2 Dimensional array to hold additional points
        double[][] morePoints = new double[orig.length + ((numToInject) * (orig.length - 1))][2];

        int index = 0;

        //loop through original array
        for (int i = 0; i < orig.length - 1; i++) {
            //copy first
            morePoints[index][0] = orig[i][0];
            morePoints[index][1] = orig[i][1];
            index++;

            for (int j = 1; j < numToInject + 1; j++) {
                //calculate intermediate x points between j and j+1 original points
                morePoints[index][0] = j * ((orig[i + 1][0] - orig[i][0]) / (numToInject + 1)) + orig[i][0];

                //calculate intermediate y points  between j and j+1 original points
                morePoints[index][1] = j * ((orig[i + 1][1] - orig[i][1]) / (numToInject + 1)) + orig[i][1];

                index++;
            }
        }

        //copy last
        morePoints[index][0] = orig[orig.length - 1][0];
        morePoints[index][1] = orig[orig.length - 1][1];
        index++;

        return morePoints;
    }

    public static double[][] doubleArrayCopy(double[][] arr) {
        //size first dimension of array
        double[][] temp = new double[arr.length][arr[0].length];

        for (int i = 0; i < arr.length; i++) {
            //Resize second dimension of array
            temp[i] = new double[arr[i].length];

            //Copy Contents
            for (int j = 0; j < arr[i].length; j++)
                temp[i][j] = arr[i][j];
        }
        return temp;
    }

    public double[][] smoothPath(double[][] path, double weight_data, double weight_smooth, double tolerance) {
        //copy array
        double[][] newPath = doubleArrayCopy(path);

        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < path.length - 1; i++) {
                for (int j = 0; j < path[i].length; j++) {
                    double aux = newPath[i][j];
                    newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
            }
        }
        return newPath;
    }

    private double[] calculateDistanceAtPoint(double[][] smoothPath) {
        //creates array to store the total distance that the robot should have traveled at that point
        double[] distanceArray = new double[smoothPath.length];
        distanceArray[0] = 0;

        for (int i = 1; i < smoothPath.length; i++) {
            distanceArray[i] = distanceArray[i - 1] + Math.hypot(Math.abs(smoothPath[i][0] - smoothPath[i - 1][0]), Math.abs(smoothPath[i - 1][1] - smoothPath[i][1]));
        }
        return distanceArray;
    }

    private double[] calculateTargetCurvatures(double[][] smoothedPath) {
        //creates an array to store all the curvatures
        double[] curvatureArray = new double[smoothedPath.length];

        // sets the curvatures at the first and last point to 0
        curvatureArray[0] = 0;
        curvatureArray[smoothedPath.length - 1] = 0;

        //loops through the array to calculate the curvatures
        for (int i = 1; i < (smoothedPath.length - 2); i++) {

            //calculates the coordinates of the points directly ahead and behind point i
            double x1 = smoothedPath[i][0] + 0.0001;
            double y1 = smoothedPath[i][1];

            double x2 = smoothedPath[i - 1][0];
            double y2 = smoothedPath[i - 1][1];

            double x3 = smoothedPath[i + 1][0];
            double y3 = smoothedPath[i + 1][1];

            //calculates the curvatures and returns the array
            double k1 = 0.5 * (Math.pow(x1, 2) + Math.pow(y1, 2) - Math.pow(x2, 2) - Math.pow(y2, 2)) / (x1 - x2);
            double k2 = (y1 - y2) / (x1 - x2);

            double b = 0.5 * (Math.pow(x2, 2) - 2 * x2 * k1 + Math.pow(y2, 2) - Math.pow(x3, 2) + 2 * x3 * k1 - Math.pow(y3, 2)) / (x3 * k2 - y3 + y2 - x2 * k2);
            double a = k1 - k2 * b;

            Double r = new Double(Math.sqrt(Math.pow(x1 - a, 2) + (Math.pow(y1 - b, 2))));
            double curvature = 0.0;
            if (!r.isNaN()) {
                curvature = 1 / r;
            }
            curvatureArray[i] = curvature;
        }
        return curvatureArray;
    }


    private double[] calculateTargetVelocities(double[][] smoothedPath, double k) {
        //creates array that holds all of the target velocities
        double[] targetVelocities = new double[smoothedPath.length];
        double a = MAXIMUM_ACCELERATION;

        //calculates the target velocities for each point
        targetVelocities[smoothedPath.length - 1] = 0; //last point target velocity is zero
        for (int i = smoothedPath.length - 2; i >= 0; i--) { //works backwards as we need to know last point's velocity to calculate current point's

            // distance from this current point to next point
            double distance = Math.hypot(Math.abs(smoothedPath[i][0] - smoothedPath[i + 1][0]), Math.abs(smoothedPath[i + 1][1] - smoothedPath[i][1]));

            // finds the smaller value between the velocity constant / the curvature and a new target velocity
            double targetVelocity = Math.min(k / calculateTargetCurvatures(smoothedPath)[i], Math.sqrt(Math.pow(targetVelocities[i + 1],2) + 2 * a * distance));
            targetVelocities[i] = targetVelocity;
        }
        return targetVelocities;
    }

    private int calculateIndexOfClosestPoint(double[][] smoothedPath) {
        // creates array in which we store the current distance to each point in our path
        double[] distances = new double[smoothedPath.length];
        for (int i = 0/*lastClosestPointIndex*/; i < smoothedPath.length; i++) {
            distances[i] = Functions.distanceFormula(smoothedPath[i][0], smoothedPath[i][1], currentCoord.getX(), currentCoord.getY());
        }

        //calculates the index of value in the array with the smallest value and returns that index
        lastClosestPointIndex = Functions.calculateIndexOfSmallestValue(distances);
        return lastClosestPointIndex;
    }


    private double calculateT(double[] lineStart, double[] lineEnd, double lookaheadDistance) {
        //current coordiante of the robot
        double[] robotVector = {currentCoord.getX(), currentCoord.getY()};

        //constants used throughout the method
        double[] d = Functions.Vectors.subtract(lineStart, lineEnd);
        double[] f = Functions.Vectors.subtract(lineStart, robotVector);
        double r = lookaheadDistance;

        double a = Functions.Vectors.dot(d, d);
        double b = 2 * Functions.Vectors.dot(f, d);
        double c = Functions.Vectors.dot(f, f) - r * r;
        double t = Double.NaN;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            // no intersection
            //will return NaN
        } else {
            // ray didn't totally miss sphere,
            // so there is a solution to
            // the equation.

            discriminant = Math.sqrt(discriminant);

            // either solution may be on or off the ray so need to test both
            // t1 is always the smaller value, because BOTH discriminant and
            // a are nonnegative.
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);


            // 3x HIT cases:
            //          -o->             --|-->  |            |  --|->
            // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

            // 3x MISS cases:
            //       ->  o                     o ->              | -> |
            // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

            if (t1 >= 0 && t1 <= 1) {
                // t1 is the intersection, and it's closer than t2
                // (since t1 uses -b - discriminant)
                // Impale, Poke
                t = t1;
            }

            if (t2 >= 0 && t2 <= 1) {
                t = t2;
            }
        }
        return t;
    }

    public double calculateCurvature(double lookaheadDistance, double[] lookaheadPoint) {
        // robot line: ax + by + c = 0
        double a = -Functions.tand(currentCoord.getHeading());
        double b = 1;
        double c = Functions.tand(currentCoord.getHeading()) * currentCoord.getX() - currentCoord.getY();

        double[] R = {currentCoord.getX(), currentCoord.getY()};
        double[] L = lookaheadPoint;
        // generate point B on robot line (for calculating sign)
        double[] B = {R[0] + Functions.cosd(currentCoord.getHeading()), R[1] + Functions.sind(currentCoord.getHeading())};

        double[] RB = {B[0] - R[0], B[1] - R[1]};
        double[] RL = {L[0] - R[0], L[1] - R[1]};

        // calculate which side of the robot line the lookahead point is on
        double side = Math.signum(Functions.Vectors.cross2D(RL, RB));

        // distance from robot line to lookahead point: d = |ax + by + c| /√(a^2 + b^2)
        double distance = Math.abs(a * lookaheadPoint[0] + b * lookaheadPoint[1] + c) / Math.sqrt(a * a + b * b);

        double curvature = 2 * side * distance / (lookaheadDistance * lookaheadDistance);
        return curvature;
    }

    public double[] calculateTargetWheelVelocities(double targetVelocity, double curvature) {

        //calculates the wheel velocity at which the wheels should be
        double leftVelocity = targetVelocity * (2 + curvature * trackWidth) / 2;
        double rightVelocity = targetVelocity * (2 - curvature * trackWidth) / 2;

        leftVelocity = Functions.rateLimiter(leftVelocity, MAXIMUM_ACCELERATION, System.nanoTime()/1e9, lastTime, lastRateLimiterOutput);
        rightVelocity = Functions.rateLimiter(rightVelocity, MAXIMUM_ACCELERATION, System.nanoTime()/1e9, lastTime, lastRateLimiterOutput);

        double[] wheelVelocities = {leftVelocity, rightVelocity};
        return wheelVelocities;
    }

    public double[]getCurrentTargetWheelVelocities(){
        return currentTargetWheelVelocities;
    }
}