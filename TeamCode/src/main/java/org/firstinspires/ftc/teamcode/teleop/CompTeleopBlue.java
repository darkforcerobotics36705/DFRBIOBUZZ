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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.robot.Launcher.TRIGGER_START_POS;
import static com.pedropathing.ivy.Ivy.*; // ivy

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
import org.firstinspires.ftc.teamcode.robot.Vision;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.Storage;



@TeleOp(name="CompTeleopBlue", group="OpMode")
public class CompTeleopBlue extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware robot = new RobotHardware();
    private Intake intake;
    private Launcher launcher;
    private Drivetrain drivetrain;
    private Vision vision;
    private Turret turret;
    private Storage storage;

    public static double PUSH_START_POS = 0;

    public Boolean isStarted;
    private void driveCommand(double yaw){
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;

            drivetrain.drive(axial, lateral, yaw);
    }
    //CHANGE FOR BIOBUZZ
    private enum AutoDriveState {
        MANUAL,
        DRIVING_TO_SHOOT,
        LOCKING_ON,
        SHOOTING,
        DRIVING_TO_INTAKE,
        INTAKING,
    }

    private AutoDriveState autoDriveState = AutoDriveState.MANUAL;
    private ElapsedTime autoDriveTimer = new ElapsedTime();

    private void setAutoDriveState(AutoDriveState newState) {
        autoDriveState = newState;
        autoDriveTimer.reset();
    }
    private double autoDriveTime = 5.5;
    private double lockOnTime = 2.0;
    //change for biobuzz
    private double shootTime = 6.5;
    private double intakeTime = 2.0;
    private boolean loopWantedClose = false;
    private boolean loopWantedFar = false;

    @Override
    public void init() {
        robot.init(hardwareMap);

        drivetrain = new Drivetrain(robot);
        drivetrain.init(hardwareMap);
        intake = new Intake(robot);
        launcher = new Launcher(robot);
        launcher.init();
        vision = new Vision(robot);
        vision.init();
        turret = new Turret(robot);
        storage = new Storage(robot);

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
        telemetry.addData("Status", "Waiting for limelight...");
        telemetry.update();

        drivetrain.follower.setStartingPose(PoseStorage.currentPose);
        robot.imu.resetYaw();


        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void init_loop(){
        LLResult result = vision.update();
        if (vision.setPose()) {
            drivetrain.follower.setPose(vision.startPose);
            telemetry.addData("Status", "Localized!");
            telemetry.addData("Pose", "%.1f, %.1f", vision.startPose.getX(), vision.startPose.getY());
        } else {
            telemetry.addData("Status", "No tag visible - aim at AprilTag");
        }
        telemetry.update();
        if (result != null && result.isValid()) {
            telemetry.addData("PedroX from Limelight: ", "%.1f", vision.limelightToPedroPose(result.getBotpose_MT2(), true)[0]);
            telemetry.addData("PedroY from Limelight: ", "%.1f", vision.limelightToPedroPose(result.getBotpose_MT2(), true)[1]);
            telemetry.addData("Pedro Heading from Limelight: ", "%.1f", vision.limelightToPedroPose(result.getBotpose_MT2(), true)[2]);
        }
    }
    @Override
    public void start(){
        resetRuntime();
    }
    @Override
    public void loop() {
        drivetrain.follower.update();
        LLResult result = vision.update();
        double distance = vision.getDistance();
        double voltage = robot.myControlHubVoltageSensor.getVoltage();
        if (result == null || !result.isValid()) {
            telemetry.addLine("Not Locked On");
        } else {
            telemetry.addLine("Locked On");
        }

        // SCHANGE FOR BIOBUZZ
        if (gamepad1.right_bumper && (autoDriveState == AutoDriveState.MANUAL) && !gamepad1.a && !gamepad1.b) {
            drivetrain.startAutoDrive(drivetrain.CLOSE_BLUE_SHOOT_POSE);
            setAutoDriveState(AutoDriveState.DRIVING_TO_SHOOT);
        } else if (gamepad1.right_bumper && (autoDriveState == AutoDriveState.MANUAL) && gamepad1.a && !gamepad1.b) {
            drivetrain.startAutoDrive(drivetrain.FAR_BLUE_SHOOT_POSE);
            setAutoDriveState(AutoDriveState.DRIVING_TO_SHOOT);
        } else if (gamepad1.right_bumper && (autoDriveState == AutoDriveState.MANUAL) && !gamepad1.a && gamepad1.b) {
            drivetrain.startAutoDrive(drivetrain.CLOSE_BLUE_SHOOT_POSE);
            setAutoDriveState(AutoDriveState.DRIVING_TO_SHOOT);
            loopWantedClose = true;
        } else if (gamepad1.right_bumper && (autoDriveState == AutoDriveState.MANUAL) && gamepad1.a && gamepad1.b) {
            drivetrain.startAutoDrive(drivetrain.FAR_BLUE_SHOOT_POSE);
            setAutoDriveState(AutoDriveState.DRIVING_TO_SHOOT);
            loopWantedFar = true;
        }

        // Cancel auto drive if driver touches the joystick
        if (drivetrain.autoDriving && (Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1)) {
            drivetrain.cancelAutoDrive();
            setAutoDriveState(AutoDriveState.MANUAL);
            loopWantedClose = false;
            loopWantedFar = false;
        }

        double yaw = 0;
        //autodrive LOOP: CHANGE FOR BIOBUZZ
        switch (autoDriveState) {
            case MANUAL:
                loopWantedClose = false;
                loopWantedFar = false;
                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                yaw = vision.getYaw(gamepad1.right_stick_x, gamepad1.left_bumper, result);
                driveCommand(yaw);
                break;

            case DRIVING_TO_SHOOT:
                telemetry.addLine("Following Path");
                if (!drivetrain.follower.isBusy()) {
                    telemetry.addLine("Arrived");
                    setAutoDriveState(AutoDriveState.LOCKING_ON);
                }
                if (autoDriveTimer.seconds() > autoDriveTime) {
                    telemetry.addLine("Auto Drive Timeout");
                    setAutoDriveState(AutoDriveState.MANUAL);
                    drivetrain.cancelAutoDrive();
                }
                break;

            case LOCKING_ON:
                if (!vision.isLockedOn()) {
                    yaw = vision.getYaw(0.0, true, result);
                    driveCommand(yaw);
                    telemetry.addLine("Aligning");
                } else {
                    telemetry.addLine("Locked On");
                    if (result != null && result.isValid()) {
                        drivetrain.follower.holdPoint(vision.limelightToPedroPose(result.getBotpose_MT2()));
                    }
                    launcher.tripleShotStarted = false;
                    setAutoDriveState(AutoDriveState.SHOOTING);
                }
                if (autoDriveTimer.seconds() > lockOnTime) {
                    telemetry.addLine("Auto Lock Timeout");
                    setAutoDriveState(AutoDriveState.MANUAL);
                    drivetrain.cancelAutoDrive();
                }
                break;

            case SHOOTING:
                if (result != null && result.isValid()) {
                    drivetrain.follower.holdPoint(vision.limelightToPedroPose(result.getBotpose_MT2()));
                }
                launcher.update(0.0, true, vision.hasTarget(), distance, voltage);
                telemetry.addLine("Shooting");
                if (launcher.tripleShotStarted && !launcher.isTripleShotActive()) {
                    telemetry.addLine("Done");
                    if (!loopWantedClose && !loopWantedFar) {
                        drivetrain.cancelAutoDrive();
                        setAutoDriveState(AutoDriveState.MANUAL);
                    } else if (loopWantedClose) {
                        drivetrain.startAutoDrive(drivetrain.CLOSE_BLUE_INTAKE_POSE, drivetrain.CLOSE_BLUE_INTAKE_CONTROL_POSE);
                        setAutoDriveState(AutoDriveState.DRIVING_TO_INTAKE);
                    } else if (loopWantedFar) {
                        drivetrain.startAutoDrive(drivetrain.FAR_BLUE_INTAKE_POSE);
                        setAutoDriveState(AutoDriveState.DRIVING_TO_INTAKE);
                    }
                }
                if (autoDriveTimer.seconds() > shootTime) {
                    telemetry.addLine("TripleShot Timeout");
                    setAutoDriveState(AutoDriveState.MANUAL);
                    drivetrain.cancelAutoDrive();
                }
                break;

            case DRIVING_TO_INTAKE:
                telemetry.addLine("Following Path");
                if (!drivetrain.follower.isBusy()) {
                    telemetry.addLine("Arrived");
                    setAutoDriveState(AutoDriveState.INTAKING);
                }
                if (autoDriveTimer.seconds() > autoDriveTime) {
                    telemetry.addLine("Auto Drive Timeout");
                    setAutoDriveState(AutoDriveState.MANUAL);
                    drivetrain.cancelAutoDrive();
                }
                break;

            case INTAKING:
                intake.update(true, false, false, false, false);
                intake.setTransfer(false, true);
                if (autoDriveTimer.seconds() > intakeTime) {
                    if (loopWantedClose) {
                        drivetrain.startAutoDrive(drivetrain.CLOSE_BLUE_SHOOT_POSE);
                    } else if (loopWantedFar) {
                        drivetrain.startAutoDrive(drivetrain.FAR_BLUE_SHOOT_POSE);
                    }
                    setAutoDriveState(AutoDriveState.DRIVING_TO_SHOOT);
                }
                break;

            default:
                telemetry.addLine("Unknown State/ No State Set");
                setAutoDriveState(AutoDriveState.MANUAL);
                drivetrain.cancelAutoDrive();
                break;
        }

        if (!drivetrain.autoDriving) {
            intake.update(gamepad2.a, gamepad2.b, gamepad1.y, gamepad2.x, launcher.isTripleShotActive());
            //intake.setTransfer(gamepad2.right_bumper, gamepad2.left_bumper);
            launcher.update(gamepad2.right_trigger, gamepad2.y, vision.hasTarget(), distance, voltage);
        }

        if (result != null && result.isValid()) {
            telemetry.addData("Location: ", vision.limelightToPedroPose(result.getBotpose_MT2(), true));
        }
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", robot.frontLeftDrive.getPower(), robot.frontRightDrive.getPower());
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", robot.backLeftDrive.getPower(), robot.backRightDrive.getPower());
        telemetry.addData("Launch State", launcher.getLaunchState());
        telemetry.addData("Current Launch Power", "%.2f", launcher.getCurrentLaunchPower());
        telemetry.addData("distance: ", distance);
        telemetry.addData("Distance to hive: ", drivetrain.getPedroDistanceToHive(PoseStorage.currentPose, "blue"));
        telemetry.update();
        telemetry.addData("Status", "Running");
    }
}
