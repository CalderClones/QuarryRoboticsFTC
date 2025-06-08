package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.ftc.Actions;

public class GrabberFiniteStateMachine {

    public boolean isSampleDetected() {
        return sampleDetected;
    }

    public void setSampleDetected(boolean sampleDetected) {
        this.sampleDetected = sampleDetected;
    }

    public double getSampleAngle() {
        return sampleAngle;
    }

    public void setSampleAngle(double sampleAngle) {
        this.sampleAngle = sampleAngle;
    }

    public Vector2d getSampleOffset() {
        return sampleOffset;
    }

    public void setSampleOffset(Vector2d sampleOffset) {
        this.sampleOffset = sampleOffset;
    }

    public enum GrabberState {
        SCANNING_REQUESTED,
        SCANNING,
        SAMPLE_DETECTED,
        MOVING_ROBOT_TO_SAMPLE,
        GRABBING_SAMPLE,
        SAMPLE_COLLECTED,
        DRIVING

    }

    ;

    private LinearOpMode opMode;
    private MecanumDrive drive;
    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;
    private TrajectoryActionBuilder trajectoryToSample;
    private TrajectoryActionBuilder returnTrajectoryFromSample;

    private boolean sampleDetected;
    private double sampleAngle;
    private Vector2d sampleOffset;

    private GrabberState grabberState;



    public GrabberFiniteStateMachine(LinearOpMode opMode, MecanumDrive drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper) {
        this.opMode = opMode;
        this.drive = drive;
        this.lift = lift;
        this.arm = arm;
        this.wrist = wrist;
        this.gripper = gripper;

        this.setSampleDetected(false);
        this.setSampleAngle(0);
        this.setSampleOffset(new Vector2d(-0.5, -3));


        grabberState = GrabberState.DRIVING;

    }

    public void update() {
        switch (grabberState) {
            case SCANNING_REQUESTED:
                //scanning has been requested - is the robot ready?
                if (lift.stationary() && arm.stationary() && wrist.stationary() && gripper.stationary()) {
                    grabberState = GrabberState.SCANNING;
                }

                if (false /*TODO: replace with condition for cancel button pressed on gamepad*/) {
                    grabberState = GrabberState.DRIVING;
                }

                break;

            case SCANNING:
                //looking for samples - webcam lets us know when we've found one
                if (isSampleDetected()) {
                    /*TODO: make the gamepad LEDs light up in the appropriate colour*/
                    grabberState = GrabberState.SAMPLE_DETECTED;
                }
                if (false /*TODO: replace with condition for cancel button pressed on gamepad*/) {
                    grabberState = GrabberState.DRIVING;
                }

                break;

            case SAMPLE_DETECTED:
                //we've found a sample - does the driver want to pick it up?
                if (true /*TODO: replace with condition for grab button pressed on gamepad*/) {
                    //TODO: sampleAngle = latest angle returned by webcam

                    Pose2d currentPose = drive.localizer.getPose();
                    Vector2d samplePosition = new Vector2d(currentPose.position.x + getSampleOffset().x, currentPose.position.y + getSampleOffset().y);
                    Pose2d samplePose = new Pose2d(samplePosition, currentPose.heading);
                    Vector2d returnPosition = new Vector2d(currentPose.position.x, currentPose.position.y);

                    trajectoryToSample = drive.actionBuilder(currentPose)
                            .strafeTo(samplePosition);

                    returnTrajectoryFromSample = drive.actionBuilder(samplePose)
                            .strafeTo(returnPosition);

                    Action driveToSample = trajectoryToSample.build();

                    Actions.runBlocking(
                            new ParallelAction(
                                    driveToSample,
                                    lift.liftToGrabbing(),
                                    wrist.wristToAngle(getSampleAngle())
                            )
                    );
                    grabberState = GrabberState.MOVING_ROBOT_TO_SAMPLE;
                }
                if (false /*TODO: replace with condition for cancel button pressed on gamepad*/) {
                    grabberState = GrabberState.DRIVING;
                }
                if (!isSampleDetected()) {
                    /*TODO: make the gamepad LEDs stop lighting up*/
                    grabberState = GrabberState.SCANNING;
                }

                break;

            case MOVING_ROBOT_TO_SAMPLE:
                //we're moving to the sample - have we reached it?
                if (true /*TODO: replace with condition for trajectory to sample has completed && lift.stationary() && wrist.stationary()*/) {

                    Actions.runBlocking(
                            gripper.gripperToClosed()
                    );
                    grabberState = GrabberState.GRABBING_SAMPLE;
                }
                if (false /*TODO: replace with condition for cancel button pressed on gamepad*/) {
                    grabberState = GrabberState.DRIVING;
                }

                break;

            case GRABBING_SAMPLE:
                //servos are moving to grasp sample - has the gripper finished closing?
                if (gripper.stationary()) {

                    Action returnFromSample = returnTrajectoryFromSample.build();
                    Actions.runBlocking(
                            new ParallelAction(
                                    returnFromSample,
                                    lift.liftToScanning(),
                                    wrist.wristToHome()
                            )
                    );
                    grabberState = GrabberState.SAMPLE_COLLECTED;
                }
                if (false /*TODO: replace with condition for cancel button pressed on gamepad*/) {
                    grabberState = GrabberState.DRIVING;
                }

                break;

            case SAMPLE_COLLECTED:
                //robot is returning to its initial condition - is it their yet?
                if (true /*TODO: replace with condition for trajectory has completed && lift.stationary() && wrist.stationary/*/) {
                    grabberState = GrabberState.DRIVING;
                }
                if (false /*TODO: replace with condition for cancel button pressed on gamepad*/) {
                    grabberState = GrabberState.DRIVING;
                }

                break;

            case DRIVING:
                //we're driving around. No automation until scan button is pressed
                if (true /*TODO: replace with condition for scan button has been pressed*/) {
                    Actions.runBlocking(
                            new ParallelAction(
                                    lift.liftToScanning(),
                                    arm.armToHorizontal(),
                                    wrist.wristToHome(),
                                    gripper.gripperToOpen()
                            )
                    );
                    grabberState = GrabberState.SCANNING_REQUESTED;
                }
                break;
        }
    }
}
