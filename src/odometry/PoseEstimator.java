package odometry;

import math.Vector;

public class PoseEstimator {

    Vector currPos;

    public PoseEstimator(Vector v) {
        currPos = new Vector(v);
    }

    public void updatePose(double distance, double heading) {

        double updatedX = distance * Math.cos(heading);
        double updatedY = distance * Math.sin(heading);
        currPos = new Vector(updatedX, updatedY);

    }

    public Vector getPose() {
        return currPos;
    }

}
