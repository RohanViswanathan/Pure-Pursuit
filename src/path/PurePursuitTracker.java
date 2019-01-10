package path;

import com.sun.tools.internal.jxc.ap.Const;
import math.Vector;
import operation.Constants;
import operation.Range;

public class PurePursuitTracker {

    private Path p;
    private double output = 0;
    private double prevOutput = 0;
    private int lastClosestPt = 0; //index in ArrayList
    private Vector currPos = new Vector(0, 0);
    private double stopDistance;
    private double lookaheadDistance;
    private double maxVelocity = Constants.maxVel;
    private double maxAccel = Constants.maxAccel;

    public PurePursuitTracker(Path p, double lookaheadDistance, double stopDistance) {
        this.p = p;
        this.lookaheadDistance = lookaheadDistance;
        this.stopDistance = stopDistance;
    }

    private double calculateFeedForward(double targetVel, double currVel) {
        double targetAcc = (targetVel - currVel)/(Constants.loopTime);
        targetAcc = Range.clip(targetAcc, -maxAccel, maxAccel);
        return (Constants.kV * rateLimiter(targetVel, Constants.maxAccel)) + (Constants.kA * targetAcc);
    }

    private double calculateFeedback(double targetVel, double currVel) {
        return Constants.kP * (targetVel - currVel);
    }

    public void update(Vector currPose, double currVel, double heading) {
        this.currPos = currPose;
        System.out.println("current pose    "+currPose);
        int closestPointIndex = getClosestPointIndex(currPos);
        Vector lookaheadPt = new Vector(0, 0);
        for (int i = closestPointIndex; i < p.robotPath.size()-1; i++) {

            Vector startPt = p.robotPath.get(i);
            Vector endPt = p.robotPath.get(i+1);
            lookaheadPt = calcVectorLookAheadPoint(startPt, endPt, currPos, lookaheadDistance);
            if (lookaheadPt.x != Double.MAX_VALUE && lookaheadPt.y != Double.MAX_VALUE) {
                break;
            }
        }
        double curvature = p.calculateCurvatureLookAheadArc(currPos, heading, lookaheadPt, lookaheadDistance);
        double leftTargetVel = getLeftTargetVelocity(p.robotPath.get(getClosestPointIndex(currPos)).getVelocity(), curvature);
        double rightTargetVel = getRightTargetVelocity(p.robotPath.get(getClosestPointIndex(currPos)).getVelocity(), curvature);
        if (leftTargetVel > maxVelocity) {
            double ratio = rightTargetVel / leftTargetVel;
            leftTargetVel = maxVelocity;
            rightTargetVel = maxVelocity * ratio;
        }
        if (rightTargetVel > Constants.maxVel) {
            double ratio = leftTargetVel / rightTargetVel;
            rightTargetVel = maxVelocity;
            leftTargetVel = maxVelocity * ratio;
        }
        System.out.println("LTV: " + leftTargetVel);
        System.out.println("RTV: " + rightTargetVel);
        double rightFF = calculateFeedForward(rightTargetVel, currVel);
        double leftFF = calculateFeedForward(leftTargetVel, currVel);
        double rightFB = calculateFeedback(rightTargetVel, currVel);
        double leftFB = calculateFeedback(leftTargetVel, currVel);
        System.out.println(leftFF);
        System.out.println(rightFF);
        double rightOutput = rightFF + rightFB;
        double leftOutput = leftFF + leftFB;
        if (getTotalPathDistance() - getCurrDistance(getClosestPointIndex(currPos)) <= stopDistance) {
            rightOutput = 0;
            leftOutput = 0;
        }
//        //run right motor to right output
//        //run left motor to left output
        System.out.print(leftOutput + "      :     " + rightOutput);
    }

    private double rateLimiter(double input, double maxRate) {
        double maxChange = Constants.loopTime * maxRate;
        output += Range.clip(input - prevOutput, -maxChange, maxChange);
        prevOutput = output;
        return output;
    }


    private double calcIntersectionPoint(Vector startPoint, Vector endPoint, Vector currPos, double lookaheadDistance) {

        Vector d = Vector.sub(endPoint, startPoint);
        Vector f = Vector.sub(startPoint, currPos);

        double a = d.dot(d);
        double b = 2*f.dot(d);
        double c = f.dot(f) - Math.pow(lookaheadDistance, 2);
        double discriminant = Math.pow(b, 2) - (4 * a * c);

        if (discriminant < 0 ){
            System.out.println("no intersections");
            return Double.NaN;
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

        System.out.println("no intersections 2");
        return Double.NaN;
    }


    private int getClosestPointIndex(Vector currPos) {
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
        if (Double.isNaN(tIntersect)) {
            return new Vector(Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            Vector vectorSegment = Vector.mult(Vector.sub(endPoint, startPoint, null).normalize(null), tIntersect);
            Vector point = Vector.add(startPoint, vectorSegment);
            return point;
        }
    }

    private double getCurrDistance(int point) {
        double distance = 0;
        for (int i = point; i >= 1; i--) {
            distance += Vector.dist(p.robotPath.get(i), p.robotPath.get(i - 1));
        }

        return distance;
    }

    private double getTotalPathDistance() {
        double distance = 0;
        for (int i = p.robotPath.size()-1; i >= 1; i--) {
            distance += Vector.dist(p.robotPath.get(i), p.robotPath.get(i - 1));
        }

        return distance;
    }


    private double getLeftTargetVelocity(double targetRobotVelocity, double curvature) { //target velocity is from closest point on path
        System.out.println("trv: " + targetRobotVelocity);
        System.out.println("curvature " + curvature);
        return targetRobotVelocity * ((2 + (Constants.robotTrack * curvature)))/2;
    }

    private double getRightTargetVelocity(double targetRobotVelocity, double curvature) {
        return targetRobotVelocity * ((2 - (Constants.robotTrack * curvature)))/2;
    }



}
