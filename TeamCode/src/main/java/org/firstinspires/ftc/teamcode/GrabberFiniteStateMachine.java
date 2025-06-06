package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.ftc.Actions;

public class GrabberFiniteStateMachine {

    public enum GrabberState{
        SCANNING_REQUESTED,
        SCANNING,
        SAMPLE_DETECTED,
        MOVING_ROBOT_TO_SAMPLE,
        GRABBING_SAMPLE,
        SAMPLE_COLLECTED,
        DRIVING

        };

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
}
