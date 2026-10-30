/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

// Ivy Core
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;

// Pedro Pathing Movements
import static com.pedropathing.ivy.pedro.PedroCommands.*;

import static org.firstinspires.ftc.teamcode.robot.ShooterConfig.distance;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.ShooterConfig;
import org.firstinspires.ftc.teamcode.robot.Vision;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.Storage;
import org.firstinspires.ftc.teamcode.robot.Trigger;



@TeleOp(name="CompTeleopRed", group="OpMode")
public class CompTeleopRed extends OpMode {
    private RobotHardware robot = new RobotHardware();
    private Intake intake;
    private Launcher launcher;
    private Drivetrain drivetrain;
    private Vision vision;
    private Turret turret;
    private Storage storage;
    private ShooterConfig shooterconfig;
    private Trigger trigger;

    public final Pose CLOSE_RED_SHOOT_POSE = new Pose(82.8, 24.0, 1.57);
    public final Pose CLOSE_RED_INTAKE_POSE = new Pose(109.9, 139.9, 0);
    public final Pose FAR_RED_SHOOT_POSE = new Pose(82.8, 115.8, 4.71);

    public Command activeCommand = null;

    //CHANGE FOR BIOBUZZ
    private enum AutoDriveState {
        MANUAL,
        DRIVING_TO_SHOOT
    }

    private AutoDriveState autoDriveState = AutoDriveState.MANUAL;
    private ElapsedTime autoDriveTimer = new ElapsedTime();

    private void setAutoDriveState(AutoDriveState newState) {
        autoDriveState = newState;
        autoDriveTimer.reset();
    }
    private double autoDriveTime = 5.5;


    @Override
    public void init() {
        Scheduler.reset();
        robot.init(hardwareMap);

        drivetrain = new Drivetrain(robot);
        drivetrain.init(hardwareMap);
        intake = new Intake(robot);
        launcher = new Launcher(robot);
        launcher.init();
        vision = new Vision();
        vision.init();
        turret = new Turret(robot);
        trigger = new Trigger(robot);
        //storage = new Storage();

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
        telemetry.addData("Status", "Waiting for limelight...");
        telemetry.update();


        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start(){
        resetRuntime();
    }
    @Override
    public void loop() {
        drivetrain.follower.update();

        if (gamepad1.right_bumper && (autoDriveState == AutoDriveState.MANUAL) && !gamepad1.a && !gamepad1.b) {
            activeCommand = drivetrain.startAutoDriveTo(CLOSE_RED_SHOOT_POSE);
            activeCommand.schedule();
            setAutoDriveState(AutoDriveState.DRIVING_TO_SHOOT);
        } else if (gamepad1.right_bumper && (autoDriveState == AutoDriveState.MANUAL) && gamepad1.a && !gamepad1.b) {
            activeCommand = drivetrain.startAutoDriveTo(FAR_RED_SHOOT_POSE);
            activeCommand.schedule();
            setAutoDriveState(AutoDriveState.DRIVING_TO_SHOOT);
        }


        // Cancel auto drive if driver touches the joystick
        if (drivetrain.autoDriving && (Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1)) {
            activeCommand.cancel();
            activeCommand = null;
            setAutoDriveState(AutoDriveState.MANUAL);
        }

        turret.updateTurret(); // set turret constantly

        switch (autoDriveState) {
            case MANUAL:

                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;
                drivetrain.move(axial, lateral, yaw);

                if (gamepad2.left_bumper){
                    intake.setPower(1);
                }
                else {
                    intake.setPower(0);
                }

                if (gamepad2.a){ // flower ramp
                    intake.lowerRamp();
                }
                else {
                    intake.retractRamp();
                }

                if (gamepad2.right_bumper){
                    trigger.fire(launcher.goodShooterConfig(), turret.goodTurretConfig());
                }
                break;

            case DRIVING_TO_SHOOT:
                telemetry.addLine("Following Path");
                if (!drivetrain.follower.isBusy()) {
                    telemetry.addLine("Arrived");
                    setAutoDriveState(AutoDriveState.MANUAL);
                }
                if (autoDriveTimer.seconds() > autoDriveTime) {
                    telemetry.addLine("Auto Drive Timeout");
                    setAutoDriveState(AutoDriveState.MANUAL);
                    activeCommand.cancel();
                    activeCommand = null;
                }
                break;

            default:
                telemetry.addLine("Unknown State/ No State Set");
                setAutoDriveState(AutoDriveState.MANUAL);
                activeCommand.cancel();
                activeCommand = null;
                break;
        }

        telemetry.addData("Front left/Right", "%4.2f, %4.2f", robot.frontLeftDrive.getPower(), robot.frontRightDrive.getPower());
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", robot.backLeftDrive.getPower(), robot.backRightDrive.getPower());
        telemetry.addData("Current Launch Power", "%.2f", launcher.getCurrentLaunchPower());
        telemetry.addData("distance: ", distance);
        telemetry.addData("Distance to hive: ", drivetrain.getPedroDistanceToHive(PoseStorage.currentPose, "red"));
        telemetry.update();
        telemetry.addData("Status", "Running");
    }
}
