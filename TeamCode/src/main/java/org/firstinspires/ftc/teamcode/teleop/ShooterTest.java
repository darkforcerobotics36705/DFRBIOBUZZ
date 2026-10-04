package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Vision;
@TeleOp(name="Shooter Test", group="OpMode")
public class ShooterTest extends OpMode {
    private double velocity = 0;
    private RobotHardware robot = new RobotHardware();
    private Launcher launcher = new Launcher(robot);
    boolean aWasPressed = false;
    boolean bWasPressed = false;
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        if (gamepad1.a && !aWasPressed) {
            velocity += 1000;
            aWasPressed = true;
        } else if (!gamepad1.a) {
            aWasPressed = false;
        }
        if (gamepad1.b && !bWasPressed) {
            velocity -= 100;
        } else if (!gamepad1.b) {
            bWasPressed = false;
        }
        launcher.setVelocityNormal(velocity);
    }
}
