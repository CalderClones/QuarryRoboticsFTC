package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OctoQuadDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Config
@Autonomous(group="test")
public final class AutoSquareTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-42, -42, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(OctoQuadDrive.class)) {
            OctoQuadDrive drive = new OctoQuadDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineToSplineHeading(new Pose2d(42,-42, Math.toRadians(90)),Math.toRadians(90))
                            .splineToSplineHeading(new Pose2d(42,42, Math.toRadians(180)),Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(-42,42, Math.toRadians(270)),Math.toRadians(270))
                            .splineToSplineHeading(new Pose2d(-42,-42, 0),0)
                            .build());
                            /*.lineToX(40)
                            .turn(Math.toRadians(90))
                            .lineToY(40)
                            .turn(Math.toRadians(90))
                            .lineToX(-40)
                            .turn(Math.toRadians(90))
                            .lineToY(-40)
                            .turn(Math.toRadians(90))
                            .waitSeconds(2)
                            .build());*/


        } else if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(SparkFunOTOSDrive.class)) {
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .lineToX(36)
                            .turn(Math.toRadians(90))
                            .lineToY(36)
                            .turn(Math.toRadians(90))
                            .lineToX(-36)
                            .turn(Math.toRadians(90))
                            .lineToY(-36)
                            .turn(Math.toRadians(90))
                            .build());

        } else if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToSplineHeading(new Pose2d(42,-42, Math.toRadians(90)),Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(42,42, Math.toRadians(180)),Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-42,42, Math.toRadians(270)),Math.toRadians(270))
                        .splineToSplineHeading(new Pose2d(-42,-42, 0),0)
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
