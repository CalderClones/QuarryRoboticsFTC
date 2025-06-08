package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.ftc.Actions;

public class GrabberFiniteStateMachine {

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

    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;

    Vector2d OffestToSample;
    Vector2d returnOffsetFromSample;

    double sampleAngle;
    Vector2d sampleOffset;

    GrabberState grabberState = GrabberState.DRIVING;

    //timer used to allow us to wait for servo movements to complete
    ElapsedTime grabberTimer = new ElapsedTime();

    double servo_delay = 250;

    public Action move_lift_to_preset;

    public GrabberFiniteStateMachine() {

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
                if (true /*TODO: replace with condition for webcam has detected sample*/) {
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
                    //TODO: generate trajectoryToSample
                    //TODO: generate returnTrajectoryFromSample

                    Actions.runBlocking(
                            new ParallelAction(
                                    //TODO: tell drivetrain to start following trajectoryToSample
                                    lift.liftToGrabbing(),
                                    wrist.wristToAngle(sampleAngle)
                            )
                    );
                    grabberState = GrabberState.MOVING_ROBOT_TO_SAMPLE;
                }
                if (false /*TODO: replace with condition for cancel button pressed on gamepad*/) {
                    grabberState = GrabberState.DRIVING;
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
                    Actions.runBlocking(
                            new ParallelAction(
                                    //TODO: tell drivetrain to start following returnTrajectoryFromSample
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
