package testing;

import math.Vector;
import odometry.PoseEstimator;
import operation.Constants;
import path.Path;
import path.PurePursuitTracker;

public class PurePursuitTest {


    public void init() {
    }

    public static void main(String[] args) { //this would run in a Looper implemented class
        //poseEstimator.updatePose((leftEncoderDist() + rightEncoderDist())/2, getAngle());
        //purePursuitTracker.update(poseEstimator.getPose(), currVel(), getAngle());
        Path p = new Path(Constants.a, Constants.b, Constants.spacing, Constants.tolerance);
        PurePursuitTracker purePursuitTracker = new PurePursuitTracker(p, 15);
        PoseEstimator poseEstimator = PoseEstimator.getInstance();
        p.addSegment(new Vector(0,0), new Vector(0, 30));
        p.addSegment(new Vector(0,30), new Vector(30, 60));
        p.addSegment(new Vector(30, 60), new Vector(30, 80));
        p.addSegment(new Vector(30, 80), new Vector(30, 100));
        p.addLastPoint();
        p.setTargetVelocities(Constants.maxVel, Constants.maxAccel, Constants.maxVelk);
        p.setCurvatures();
        purePursuitTracker.update(poseEstimator.getPose(), 0, 10);
    }

}
