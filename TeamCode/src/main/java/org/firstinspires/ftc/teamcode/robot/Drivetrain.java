package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.firstinspires.ftc.teamcode.auto.FarAuto.shootAngle;
import com.pedropathing.api.PoseFactory;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drivetrain {

    private RobotHardware robot;
    public Follower follower;
    private final PoseFactory p = PoseFactory.degrees(); // add this
    public boolean autoDriving = false;
    // CHANGE FOR BIOBUZZ
    public final Pose CLOSE_BLUE_SHOOT_POSE = new Pose(58.78, 84.27,  Math.toRadians(135));
    public final Pose CLOSE_BLUE_INTAKE_POSE = new Pose(129.8,  62.4,     Math.toRadians(35)).mirror();
    public final Pose CLOSE_BLUE_INTAKE_CONTROL_POSE = new Pose(95.4, 48.7, Math.toRadians(0)).mirror();
    public final Pose FAR_BLUE_SHOOT_POSE = new Pose(86.7, 20.3, Math.toRadians(shootAngle)).mirror();
    public final Pose FAR_BLUE_INTAKE_POSE = new Pose(142.58, 9.04,  Math.toRadians(0)).mirror();
    public final Pose CLOSE_RED_SHOOT_POSE = new Pose(58.78, 84.27,  Math.toRadians(140)).mirror();
    public final Pose CLOSE_RED_INTAKE_POSE = new Pose(129.8,  62.4,     Math.toRadians(35));
    public final Pose CLOSE_RED_INTAKE_CONTROL_POSE = new Pose(95.4, 48.7, Math.toRadians(0));
    public final Pose FAR_RED_SHOOT_POSE = new Pose(86.7, 20.3, Math.toRadians(shootAngle));
    public final Pose FAR_RED_INTAKE_POSE = new Pose(132.57660626029653, 9.043956043956046,  Math.toRadians(0));

    public Drivetrain(RobotHardware robot) {
        this.robot = robot;
    }

    /** Call every loop with joystick inputs (and yaw already overridden if lock-on is active) */
    public void drive(double axial, double lateral, double yaw) {
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize so no wheel exceeds 100%
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        robot.frontLeftDrive.setPower(frontLeftPower);
        robot.frontRightDrive.setPower(frontRightPower);
        robot.backLeftDrive.setPower(backLeftPower);
        robot.backRightDrive.setPower(backRightPower);
    }
    public void cancelAutoDrive() {
        follower.breakFollowing();
        autoDriving = false;
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public void driveForward(double power, long ms) throws InterruptedException {
        drive(power, 0, 0);
        Thread.sleep(ms);
        stop();
    }

    public void init(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
    }

    public void startAutoDrive(Pose targetPose) {
        // Get current robot pose from Pedro's odometry
        Pose currentPose = follower.getPose();

        // Build a straight line path from here to the target
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(path, false);
        autoDriving = true;
    }

    public void startAutoDrive(Pose targetPose, Pose controlPoint) {
        Pose currentPose = follower.getPose();
        PathChain path = follower.pathBuilder()
                .addPath(new BezierCurve(currentPose, controlPoint, targetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading(), 0.6)
                .build();
        follower.followPath(path, true);
        autoDriving = true;
    }

    public void initStartingPoint(Pose startPose) {
        follower.setStartingPose(startPose);
    }
    public void holdPoint(Pose targetPose) {
        follower.holdPoint(targetPose);
    }

    public Command driveWith(PathChain path, Command... others) {
        Command follow = sequential(
                instant(() -> follower.followPath(path, true)),
                waitUntil(() -> !follower.isBusy())
        );

        Command[] all = new Command[others.length + 1];
        all[0] = follow;
        System.arraycopy(others, 0, all, 1, others.length);
        return parallel(all);
    }
    // public void move(double y, double x, double yaw){ // teleop
    //    follower.move(y,x,yaw);
    //    follower.update();
    //}
    public double[] getPedroDistanceToHive(Pose currentPose, String allianceColor){
        double[] Hive1 = {0.0, 0.0};
        double[] Hive2 = {0.0, 0.0};

        if (allianceColor.equals("red")){
            Hive1 = new double[] {58.746, 24.076}; // replace with red hive coordinates
            Hive2 = new double[] {58.746, 115.815};
        }
        else{
            Hive1 = new double[] {82.754, 24.076}; // replace with blue hive coordinates
            Hive2 = new double[] {82.754, 115.815};
        }
        double[] vector1 = {Hive1[0] - currentPose.getX(), Hive1[1] - currentPose.getY()};
        double[] vector2 = {Hive2[0] - currentPose.getX(), Hive2[1] - currentPose.getY()};
        if (Math.hypot(vector1[0], vector1[1]) < Math.hypot(vector2[0], vector2[1])) {
            return vector1;
        }
        else{
            return vector2;
        }
    }


}
