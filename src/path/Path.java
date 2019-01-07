package path;

import math.Vector;
import operation.Range;
import operation.Sign;

import java.util.ArrayList;
import java.util.NavigableMap;

public class Path {

    double spacing = 6; //spacing between points
    double distance;
    ArrayList<Vector> newPoints = new ArrayList<>();
    double prevTime = 0;
    double output = 0;
    double prevOutput = 0;
    int lastClosestPt = 0;

    public Path() {
        distance = 0;
    }

    public void injectPoints(Vector startPt, Vector endPt) {

        Vector vector = new Vector(Vector.sub(startPt, endPt, null));
        double num_pts_that_fit = Math.ceil(vector.norm() / spacing);
        vector.normalize();
        vector.mult(spacing);
        for (int i = 0; i < num_pts_that_fit; i++) {
            vector.mult(i);
            newPoints.add(Vector.add(startPt, vector, null));
        }
        newPoints.add(endPt);

    }

    public double [][] makeArray(ArrayList<Vector> pts) {

        double [][] path = new double [2][pts.size()];
        for (int i = 0; i < pts.size(); i++) {
            path[0][i] = pts.get(i).x;
            path[1][i] = pts.get(i).y;
        }

        return path;

    }

    public ArrayList<Vector> makeList(double [][] pts) {

        ArrayList<Vector> path = new ArrayList<>();
        for (int i = 0; i < pts[0].length; i ++){
            path.add(new Vector(pts[0][i], pts[1][i]));
        }

        return path;
    }

    public double [][] smooth(double [][] path, double a, double b, double tolerance) {

        double [][] newPath = path;
        double change = tolerance;

        while (change >= 0) {
            change = 0;
            for (int i = 1; i < path.length - 1; i++) {
                for (int j = 1; j < path[i].length; j++) {
                    double aux = newPath[i][j];
                    newPath[i][j] += a * (path[i][j] - newPath[i][j]) + b * (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
            }
        }

        return newPath;

    }

    public double getCurrDistance(ArrayList<Vector> path, int point) {
        double distance = 0;
        for (int i = point; i >= 0; i--) {
             distance += Vector.dist(path.get(i), path.get(i - 1));
        }

        return distance;
    }

    public double calculatePathCurvature(ArrayList<Vector> path, int point) {
        Vector pt = new Vector(path.get(point));
        Vector prevPt = new Vector(path.get(point - 1));
        Vector nextPt = new Vector(path.get(point + 1));

        double productOfSides = Vector.dist(pt, prevPt) * Vector.dist(pt, nextPt) * Vector.dist(nextPt, prevPt);
        double semiPerimeter = (Vector.dist(pt, prevPt) + Vector.dist(pt, nextPt) + Vector.dist(nextPt, prevPt))/2;
        double triangleArea = Math.sqrt(semiPerimeter * (semiPerimeter - Vector.dist(pt, prevPt)) * (semiPerimeter - Vector.dist(pt, nextPt)) * (semiPerimeter - Vector.dist(nextPt, prevPt)));

        double radius = (productOfSides)/(4 * triangleArea);
        double curvature = 1/radius;

        return curvature;
    }

    public double calculateMaxVelocity(ArrayList<Vector> path, int point, double pathMaxVel, double k) {
        double curvature = calculatePathCurvature(path, point);
        return Math.min(pathMaxVel, k/curvature); //k is a constant (generally between 1-5 based on how quickly you want to make the turn)
    }

    public void setTargetVelocities(ArrayList<Vector> path, double maxVel, double maxAccel, double k) {
        path.get(path.size() - 1).setVelocity(0);
        for (int i = path.size() - 2; i >= 0; i--) {
            distance = Vector.dist(path.get(i+1), path.get(i));
            double maxReachableVel = Math.sqrt(Math.pow(path.get(i+1).getVelocity(),2) + (2 * maxAccel * distance));
            path.get(i).setVelocity(Math.min(calculateMaxVelocity(path, i, maxVel, k), maxReachableVel));
        }
    }


    public double rateLimiter(double input, double maxRate, double systemTime) {
        double deltaTime = systemTime - prevTime;
        double maxChange = deltaTime * maxRate;
        output += Range.clip(input - prevOutput, -maxChange, maxChange);
        prevOutput = output;
        prevTime = systemTime;
        return output;
    }

    public int getClosestPoint(Vector currPos, ArrayList<Vector> path) {
        double shortestDistance = 100000000;
        int closestPoint = 0;
        for (int i = lastClosestPt; i < path.size(); i++) {
            if (Vector.dist(path.get(i), currPos) < shortestDistance) {
                closestPoint = i;
                shortestDistance = Vector.dist(path.get(i), currPos);
            }
        }
        lastClosestPt = closestPoint;
        return closestPoint;

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

    public Vector calcVectorLookAheadPoint(Vector startPoint, Vector endPoint, Vector currPos, double lookaheadDistance) {
        double tIntersect = calcIntersectionPoint(startPoint, endPoint, currPos, lookaheadDistance);
        Vector point = Vector.add(startPoint, Vector.mult(Vector.sub(endPoint, startPoint, null), tIntersect));
        return point;
    }

    public double calcCurvatureLookAheadArc(Vector currPos, double heading, Vector lookahead, double lookaheadDistance) {
        double a = -Math.tan(heading);
        double b = 1;
        double c = (Math.tan(heading)*currPos.x) - currPos.y;
        double x = Math.abs(a * lookahead.x + b * lookahead.y + c)/Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
        double cross = (Math.sin(heading) * (lookahead.x - currPos.x)) - (Math.cos(heading) * (lookahead.y - currPos.y));
        double side = Sign.getSign(cross);
        double curvature = (2 * x)/(Math.pow(lookaheadDistance, 2));
        return curvature * side;
    }


    public static void main (String[] args) {
        Path path = new Path();
        Vector start = new Vector(1,3);
        Vector end = new Vector(3,9);
        Vector currPos = new Vector(-3, 5);
        //System.out.print(path.calcIntersectionPoint(start, end, currPos, 5));
        System.out.print(path.calcVectorLookAheadPoint(start, end, currPos, 5).x);
        System.out.print(path.calcVectorLookAheadPoint(start, end, currPos, 5).y);

    }



}
