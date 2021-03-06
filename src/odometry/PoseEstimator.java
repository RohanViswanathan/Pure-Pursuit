package odometry;

import math.*;

public class PoseEstimator /*implements Loop*/ {
    private static PoseEstimator instance;
    private Twist velocity;
    private RigidTransform pose;
    private RigidTransform prevPose;
    private double prevLeftDist = 0;
    private double prevRightDist = 0;
    //private DriveTrain driveTrain = DriveTrain.getInstance();

    private PoseEstimator(){
        reset(new RigidTransform());
}

    public static PoseEstimator getInstance(){
        return instance == null ? instance = new PoseEstimator() : instance;
    }

    public Vector getPose() {
        return new Vector(pose.getTranslation().getX(), pose.getTranslation().getY());
    }

    public Twist getVelocity() {
        return velocity;
    }

    public void reset(RigidTransform startingPose){
        velocity = new Twist();
        pose = startingPose;
        prevPose = new RigidTransform();
    }

    //@Override
    public void init(double timestamp) {
        //prevLeftDist = driveTrain.getLeftDistance();
        //prevRightDist = driveTrain.getRightDistance();
        reset(new RigidTransform());
    }

    //@Override
    public void update(double timestamp) {
        double leftDist = 0 /*driveTrain.getLeftDistance()*/;
        double rightDist = 0 /*driveTrain.getRightDistance()*/;
        double deltaLeftDist = leftDist - prevLeftDist;
        double deltaRightDist = rightDist - prevRightDist;
        Rotation deltaHeading = prevPose.getRotation().inverse().rotate(new Rotation(0, 0)/*driveTrain.getRotationAngle()*/);
        //Use encoders + gyro to determine our velocity
        velocity = Kinematics.forwardKinematics(deltaLeftDist, deltaRightDist,
                deltaHeading.radians());
        //use velocity to determine our pose
        pose = Kinematics.integrateForwardKinematics(prevPose, velocity);
        //update for next iteration
        prevLeftDist = leftDist;
        prevRightDist = rightDist;
        prevPose = pose;
        //System.out.println("Pose:   " + pose);
    }

    //@Override
    public void end(double timestamp) {

    }
}
