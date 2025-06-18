package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;
import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.GrabberFiniteStateMachine;
import org.firstinspires.ftc.teamcode.Gripper;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Wrist;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class QuarryRoboticsMotorEncoderTest extends LinearOpMode {

    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;

    private  GrabberFiniteStateMachine stateMachine;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        gripper = new Gripper(hardwareMap);
        //stateMachine = new GrabberFiniteStateMachine(this, drive, lift, arm, wrist, gripper);

        telemetry.addLine("Pausing to allow OTOS to initialise");
        telemetry.update();
        sleep(1000);
        telemetry.addLine("OTOS should be initialised");
        telemetry.update();
        telemetry.clear();
        lift.liftMotor.setPower(0);
        lift.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wrist.wristMotor.setPower(0);
        wrist.wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.armMotor.setPower(0);
        arm.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("LiftTarget", lift.get_target_height());
            telemetry.addData("LiftHeight", lift.getCurrentHeight());
            telemetry.addData("LiftEncoder", lift.liftMotor.getCurrentPosition());
            telemetry.addData("ArmEncoder", arm.armMotor.getCurrentPosition());
            telemetry.addData("WristEncoder", wrist.wristMotor.getCurrentPosition());
            telemetry.update();
            sleep(50);
        }

    }
}


