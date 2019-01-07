package path;

import math.Vector;

import java.util.ArrayList;

public class Path {

    double spacing = 6; //spacing between points
    double distance;
    ArrayList<Vector> newPoints = new ArrayList<>();

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

    public double calculateCurvature(ArrayList<Vector> path, int point) {
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

}
