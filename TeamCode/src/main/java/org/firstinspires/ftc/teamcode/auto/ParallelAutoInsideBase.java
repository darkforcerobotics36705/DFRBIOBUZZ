package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.ivy.Scheduler;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.Path;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

public abstract class ParallelAutoInsideBase extends LinearOpMode {
    @Autonomous

public class ExampleAuto extends OpMode {

    private Follower follower; // Add this
        private final PoseFactory p = PoseFactory.degrees();
        private final Pose startPose = p.of(56,8,90);
        private final Pose controlPose = p.of(67.9, 72.8, 100);
        private final Pose park = p.of(50.5,130,167);
        private Path park() {
            return curve(startPose,park).linear(startPose,park);
        }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }
    @Override
    public void start() {
            schedule(follow(follower, park));
    }

        private Command follow(Follower follower, Pose park) {
            return null;
        }

        @Override
    public void loop() {
                follower.update();
                Scheduler.execute();
            }
        }
}
