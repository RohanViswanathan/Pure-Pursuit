package path;

import math.Vector;
import operation.Constants;
import operation.Range;

import java.util.ArrayList;

public class PurePursuitTracker {

    private Path p;
    double output = 0;
    double prevOutput = 0;
    int lastClosestPt = 0; //index in ArrayList
    Vector currPos = new Vector(0, 0);
    double currVel = 0;
    double lookaheadDistance;


    public PurePursuitTracker(Path p, double lookaheadDistance) {
        this.p = p;
        this.lookaheadDistance = lookaheadDistance;
    }

    public double calculateFeedForward(double targetVel, double currVel, double currTime) {
        double targetAcc = (targetVel - currVel)/(Constants.loopTime);
        return (Constants.kV * targetVel) + (Constants.kA * targetAcc);
    }

    public double calculateFeedback(double targetVel, double currVel) {
        return Constants.kP * (targetVel - currVel);
    }

    public void update(Vector currPose, double currVel) {
        this.currPos = currPose;
        this.currVel = currVel;
        Vector startPt = p.robotPath.get(getClosestPointIndex(currPos));
        Vector endPt = p.robotPath.get(getClosestPointIndex(currPos) + 1);
        Vector lookaheadPt = calcVectorLookAheadPoint(startPt, endPt, currPos, lookaheadDistance);

    }

    public double rateLimiter(double input, double maxRate) {
        double maxChange = Constants.loopTime * maxRate;
        output += Range.clip(input - prevOutput, -maxChange, maxChange);
        prevOutput = output;
        return output;
    }


    public double calcIntersectionPoint(Vector startPoint, Vector endPoint, Vector currPos, double lookaheadDistance) {

        Vector d = Vector.sub(endPoint, startPoint);
        Vector f = Vector.sub(startPoint, currPos);

        double a = d.dot(d);
        double b = 2*f.dot(d);
        double c = f.dot(f) - Math.pow(lookaheadDistance, 2);
        double discriminant = Math.pow(b, 2) - (4 * a * c);

        if (discriminant < 0 ){
            return 0;
        }

        else {
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b - discriminant)/(2 * a);
            double t2 = (-b + discriminant)/(2 * a);

            if (t1 >= 0 && t1 <= 1) {
                System.out.println("t1");
                return t1;
            }
            if (t2 >= 0 && t2 <= 1) {
                System.out.println("t2");
                return t2;
            }

        }

        return 0;
    }


    public int getClosestPointIndex(Vector currPos) {
        double shortestDistance = Double.MAX_VALUE;
        int closestPoint = 0;
        for (int i = lastClosestPt; i < p.robotPath.size(); i++) {
            if (Vector.dist(p.robotPath.get(i), currPos) < shortestDistance) {
                closestPoint = i;
                shortestDistance = Vector.dist(p.robotPath.get(i), currPos);
            }
        }
        lastClosestPt = closestPoint;
        return closestPoint;

    }

    public Vector calcVectorLookAheadPoint(Vector startPoint, Vector endPoint, Vector currPos, double lookaheadDistance) {
        double tIntersect = calcIntersectionPoint(startPoint, endPoint, currPos, lookaheadDistance);
        Vector point = Vector.add(startPoint, Vector.mult(Vector.sub(endPoint, startPoint, null), tIntersect));
        return point;
    }


    public double getCurrDistance(ArrayList<Vector> path, int point) {
        double distance = 0;
        for (int i = point; i >= 1; i--) {
            distance += Vector.dist(path.get(i), path.get(i - 1));
        }

        return distance;
    }

    public double getLeftTargetVelocity(double targetRobotVelocity, double curvature) { //target velocity is from closest point on path
        return targetRobotVelocity * ((2 + (Constants.robotTrack * curvature)))/2;
    }


    public double getRightTargetVelocity(double targetRobotVelocity, double curvature) {
        return targetRobotVelocity * ((2 - (Constants.robotTrack * curvature)))/2;
    }



}
