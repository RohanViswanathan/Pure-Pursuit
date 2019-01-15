package path;

import operation.Constants;
import operation.Sign;

import java.util.ArrayList;
import java.util.Arrays;


public class Path {

    double spacing = Constants.spacing; //spacing between points in inches
    double distance; //in inches
    double a, b, tolerance;
    ArrayList<frc.team3256.robot.math.Vector> robotPath = new ArrayList<>();

    public Path(double a, double b, double tolerance) {
        this.a = a;
        this.b = b;
        this.tolerance = tolerance;
        distance = 0;
    }

    private void injectPoints(frc.team3256.robot.math.Vector startPt, frc.team3256.robot.math.Vector endPt, ArrayList<frc.team3256.robot.math.Vector> temp) {

        frc.team3256.robot.math.Vector vector = new frc.team3256.robot.math.Vector(frc.team3256.robot.math.Vector.sub(endPt, startPt, null));
        double num_pts_that_fit = Math.ceil(vector.norm() / spacing);
        frc.team3256.robot.math.Vector unitVector = vector.normalize(null);
        unitVector.mult(vector.norm() / num_pts_that_fit);
        for (int i = 0; i < num_pts_that_fit; i++) {
            frc.team3256.robot.math.Vector newVector = frc.team3256.robot.math.Vector.mult(unitVector, i, null);
            temp.add(frc.team3256.robot.math.Vector.add(startPt, newVector, null));
        }
        temp.add(endPt);
    }

    private double [][] makeArray(ArrayList<frc.team3256.robot.math.Vector> pts) {

        double [][] path = new double [pts.size()][2];
        for (int i = 0; i < pts.size(); i++) {
            path[i][0] = pts.get(i).x;
            path[i][1] = pts.get(i).y;
        }

        return path;

    }

    private ArrayList<frc.team3256.robot.math.Vector> makeList(double [][] pts) {

        ArrayList<frc.team3256.robot.math.Vector> path = new ArrayList<>();
        for (int i = 0; i < pts.length; i ++){
            path.add(new frc.team3256.robot.math.Vector(pts[i][0], pts[i][1]));
        }

        return path;
    }

    private double [][] doubleArrayCopy(double [][] array) {
        double [][] newArray = new double[array.length][];
        for(int i = 0; i < array.length; i++)
            newArray[i] = Arrays.copyOf(array[i], array[i].length);
        return newArray;
    }

    private ArrayList<frc.team3256.robot.math.Vector> smooth(ArrayList<frc.team3256.robot.math.Vector> vectorPath, double a, double b, double tolerance) {

        double [][] path = makeArray(vectorPath);

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
        return makeList(newPath);

    }

    public void setCurvature() {
        for (int i = 1; i < robotPath.size()-1; i++){
            robotPath.get(i).setCurvature(calculatePathCurvature(robotPath, i));
        }
    }


    private double calculatePathCurvature(ArrayList<frc.team3256.robot.math.Vector> path, int point) {
        frc.team3256.robot.math.Vector pt = new frc.team3256.robot.math.Vector(path.get(point));
        frc.team3256.robot.math.Vector prevPt = new frc.team3256.robot.math.Vector(path.get(point - 1));
        frc.team3256.robot.math.Vector nextPt = new frc.team3256.robot.math.Vector(path.get(point + 1));

        double productOfSides = frc.team3256.robot.math.Vector.dist(pt, prevPt) * frc.team3256.robot.math.Vector.dist(pt, nextPt) * frc.team3256.robot.math.Vector.dist(nextPt, prevPt);
        double semiPerimeter = (frc.team3256.robot.math.Vector.dist(pt, prevPt) + frc.team3256.robot.math.Vector.dist(pt, nextPt) + frc.team3256.robot.math.Vector.dist(nextPt, prevPt))/2;
        double triangleArea = Math.sqrt(semiPerimeter * (semiPerimeter - frc.team3256.robot.math.Vector.dist(pt, prevPt)) * (semiPerimeter - frc.team3256.robot.math.Vector.dist(pt, nextPt)) * (semiPerimeter - frc.team3256.robot.math.Vector.dist(nextPt, prevPt)));

        double radius = (productOfSides)/(4 * triangleArea);
        double curvature = 1/radius;

        return curvature;
    }

    private double calculateMaxVelocity(ArrayList<frc.team3256.robot.math.Vector> path, int point, double pathMaxVel, double k) {
        if (point > 0) {

            double curvature = calculatePathCurvature(path, point);
            return Math.min(pathMaxVel, k/curvature); //k is a constant (generally between 1-5 based on how quickly you want to make the turn)

        }
        return pathMaxVel;
    }

    public void setTargetVelocities(double maxVel, double maxAccel, double k) {
        robotPath.get(robotPath.size() - 1).setVelocity(0);
        for (int i = robotPath.size() - 2; i >= 0; i--) {
            distance = frc.team3256.robot.math.Vector.dist(robotPath.get(i+1), robotPath.get(i));
            double maxReachableVel = Math.sqrt(Math.pow(robotPath.get(i+1).getVelocity(),2) + (2 * maxAccel * distance));
            robotPath.get(i).setVelocity(Math.min(calculateMaxVelocity(robotPath, i, maxVel, k), maxReachableVel));
        }
    }


    public double calculateCurvatureLookAheadArc(frc.team3256.robot.math.Vector currPos, double heading, frc.team3256.robot.math.Vector lookahead, double lookaheadDistance) {
        double a = -Math.tan(heading);
        double b = 1;
        double c = (Math.tan(heading)*currPos.x) - currPos.y;
        double x = Math.abs(a * lookahead.x + b * lookahead.y + c)/Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
        double cross = (Math.sin(heading) * (lookahead.x - currPos.x)) - (Math.cos(heading) * (lookahead.y - currPos.y));
        double side = Sign.getSign(cross);
        double curvature = (2 * x)/(Math.pow(lookaheadDistance, 2));
        return curvature * side;
    }


    public void addSegment(frc.team3256.robot.math.Vector start, frc.team3256.robot.math.Vector end) {
        ArrayList<frc.team3256.robot.math.Vector> injectTemp = new ArrayList<>();
        injectPoints(start, end, injectTemp);
        System.out.println(injectTemp);
        //ArrayList<Vector> smoothTemp = smooth(injectTemp, a, b, tolerance);
        //System.out.println(smoothTemp.size());
        if (robotPath.size() == 0) {
            for (int i = 0; i < injectTemp.size(); i++) {
                robotPath.add(injectTemp.get(i));
            }
        }
        else {
            for (int i = 0; i < injectTemp.size() - 1; i++) {
                robotPath.add(injectTemp.get(i));
            }
        }
        for (frc.team3256.robot.math.Vector v : robotPath) {
            System.out.println(v);
        }
        System.out.println("RPath Size: " + robotPath.size());
    }


    public static void main (String[] args) {
        Path p = new Path(1, 0.78, 0.001);
        PurePursuitTracker purePursuitTracker = new PurePursuitTracker(p, 5, 0);
        frc.team3256.robot.math.Vector start = new frc.team3256.robot.math.Vector(0,0);
        frc.team3256.robot.math.Vector end = new frc.team3256.robot.math.Vector(0,100);
        frc.team3256.robot.math.Vector currPos = new frc.team3256.robot.math.Vector(2.88, .84);
        //System.out.print(path.calcIntersectionPoint(start, end, currPos, 5));
        System.out.println(purePursuitTracker.calcVectorLookAheadPoint(start, end, currPos, 10));

    }



}
