package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@Disabled
@TeleOp(group = "drive")
public class QuarryRoboticsMotorSpeedProfiling extends LinearOpMode {

    public static int speed = 10;


    private final String alliance = "BOTH";

    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        speed = 0;


while(!isStarted() && !isStopRequested())
        {
            telemetry.update();
            sleep(50);
        }

        ElapsedTime timer = new ElapsedTime();

        while (speed < 11) {
            timer.reset();

            Vector2d input = new Vector2d(
                    speed / 10.0,
                    0);

            drive.setDrivePowers(new PoseVelocity2d(
                    input,
                    0));



            while (timer.seconds() < 3) {
                TelemetryPacket packet = new TelemetryPacket();
                //Log telemetry for all four wheels
                packet.put("Speed", speed);
                packet.put("Front Left Speed", drive.leftFront.getVelocity());
                packet.put("Front Right Speed", drive.rightFront.getVelocity());
                packet.put("Back Left Speed", drive.leftBack.getVelocity());
                packet.put("Back Right Speed", drive.rightBack.getVelocity());
                dashboard.sendTelemetryPacket(packet);
            }
            speed++;
        }

    }
}


