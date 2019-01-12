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
        Path p = new Path(Constants.a, Constants.b, Constants.tolerance);
        PurePursuitTracker purePursuitTracker = new PurePursuitTracker(p, 12*15, 3);
        PoseEstimator poseEstimator = new PoseEstimator(new Vector(0,0));
        p.addSegment(new Vector(0,0), new Vector(0, 1000));
        p.setTargetVelocities(Constants.maxVel, Constants.maxAccel, Constants.maxVelk);
        p.setCurvature();
        poseEstimator.updatePose(3, 10);
        purePursuitTracker.prevLeftOutput = 0;
        purePursuitTracker.prevRightOutput = 0;
        purePursuitTracker.leftOutput = 0;
        purePursuitTracker.rightOutput = 0;
        purePursuitTracker.update(poseEstimator.getPose(), 0, 10);
    }

}
