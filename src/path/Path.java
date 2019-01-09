package path;

import math.Vector;
import operation.Constants;
import operation.Sign;

import java.util.ArrayList;
import java.util.Arrays;


public class Path {

    double spacing = Constants.spacing; //spacing between points in inches
    double distance; //in inches
    double a, b, tolerance;
    ArrayList<Vector> robotPath = new ArrayList<>();

    public Path(double a, double b, double tolerance) {
        this.a = a;
        this.b = b;
        this.tolerance = tolerance;
        distance = 0;
    }

    private void injectPoints(Vector startPt, Vector endPt, ArrayList<Vector> temp) {

        Vector vector = new Vector(Vector.sub(startPt, endPt, null));
        double num_pts_that_fit = Math.ceil(vector.norm() / spacing);
        vector.normalize();
        vector.mult(spacing);
        for (int i = 0; i < num_pts_that_fit; i++) {
            vector.mult(i);
            temp.add(Vector.add(startPt, vector, null));
        }
        temp.add(endPt);
    }

    private double [][] makeArray(ArrayList<Vector> pts) {

        double [][] path = new double [2][pts.size()];
        for (int i = 0; i < pts.size(); i++) {
            path[0][i] = pts.get(i).x;
            path[1][i] = pts.get(i).y;
        }

        return path;

    }

    private ArrayList<Vector> makeList(double [][] pts) {

        ArrayList<Vector> path = new ArrayList<>();
        for (int i = 0; i < pts[0].length; i ++){
            path.add(new Vector(pts[0][i], pts[1][i]));
        }

        return path;
    }

    private double [][] doubleArrayCopy(double [][] array) {
        double [][] newArray = new double[array.length][];
        for(int i = 0; i < array.length; i++)
            newArray[i] = Arrays.copyOf(array[i], array[i].length);
        return newArray;
    }

    private double [][] smooth(double [][] path, double a, double b, double tolerance) {

        double [][] newPath = doubleArrayCopy(path);
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

    public void setCurvature() {
        for (int i = 0; i < robotPath.size(); i++){
            robotPath.get(i).setCurvature(calculatePathCurvature(robotPath, i));
        }
    }


    private double calculatePathCurvature(ArrayList<Vector> path, int point) {
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

    private double calculateMaxVelocity(ArrayList<Vector> path, int point, double pathMaxVel, double k) {
        double curvature = calculatePathCurvature(path, point);
        return Math.min(pathMaxVel, k/curvature); //k is a constant (generally between 1-5 based on how quickly you want to make the turn)
    }

    public void setTargetVelocities(double maxVel, double maxAccel, double k) {
        robotPath.get(robotPath.size() - 1).setVelocity(0);
        for (int i = robotPath.size() - 2; i >= 0; i--) {
            distance = Vector.dist(robotPath.get(i+1), robotPath.get(i));
            double maxReachableVel = Math.sqrt(Math.pow(robotPath.get(i+1).getVelocity(),2) + (2 * maxAccel * distance));
            robotPath.get(i).setVelocity(Math.min(calculateMaxVelocity(robotPath, i, maxVel, k), maxReachableVel));
        }
    }


    public double calculateCurvatureLookAheadArc(Vector currPos, double heading, Vector lookahead, double lookaheadDistance) {
        double a = -Math.tan(heading);
        double b = 1;
        double c = (Math.tan(heading)*currPos.x) - currPos.y;
        double x = Math.abs(a * lookahead.x + b * lookahead.y + c)/Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
        double cross = (Math.sin(heading) * (lookahead.x - currPos.x)) - (Math.cos(heading) * (lookahead.y - currPos.y));
        double side = Sign.getSign(cross);
        double curvature = (2 * x)/(Math.pow(lookaheadDistance, 2));
        return curvature * side;
    }


    public void addSegment(Vector start, Vector end) {
        ArrayList<Vector> injectTemp = new ArrayList<>();
        injectPoints(start, end, injectTemp);
        ArrayList<Vector> smoothTemp = makeList(smooth(makeArray(injectTemp), a, b, tolerance));
        for (int i = robotPath.size()-1; i < robotPath.size() + smoothTemp.size() - 2; i++) {
            robotPath.add(smoothTemp.get(i));
        }
    }


    public static void main (String[] args) {
        Path p = new Path(1, 2, 3);
        PurePursuitTracker purePursuitTracker = new PurePursuitTracker(p, 5, 0);
        Vector start = new Vector(1,3);
        Vector end = new Vector(3,9);
        Vector currPos = new Vector(-3, 5);
        //System.out.print(path.calcIntersectionPoint(start, end, currPos, 5));
        System.out.print(purePursuitTracker.calcVectorLookAheadPoint(start, end, currPos, 5).x);
        System.out.print(purePursuitTracker.calcVectorLookAheadPoint(start, end, currPos, 5).y);

    }



}
