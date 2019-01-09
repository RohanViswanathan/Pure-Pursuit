package testing;

import math.Vector;
import odometry.PoseEstimator;
import operation.Constants;
import path.Path;
import path.PurePursuitTracker;

public class PurePursuitTest {

    Path p = new Path(Constants.a, Constants.b, Constants.tolerance);
    PurePursuitTracker purePursuitTracker = new PurePursuitTracker(p, 12, 3);
    PoseEstimator poseEstimator = new PoseEstimator(new Vector(0,0));

    public void init() {
        p.addSegment(new Vector(0,0), new Vector(0, 12));
        p.setTargetVelocities(Constants.maxVel, Constants.maxAccel, Constants.maxVelk);
        p.setCurvature();
    }

    public void run() { //this would run in a Looper implemented class
        //poseEstimator.updatePose((leftEncoderDist() + rightEncoderDist())/2, getAngle());
        //purePursuitTracker.update(poseEstimator.getPose(), getCurrentVelocity(), getAngle());
    }

}
