package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 13.28)
                .setDimensions(18,18)
                .build();

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 13.28)
                .setDimensions(18,18)
                .build();

        CameraCentre cameraCentre = new CameraCentre(meepMeep, myBot, new Vector2d(0,0), new Vector2d(0,0));
        GripperCentre gripperCentre = new GripperCentre(meepMeep, myBot, new Vector2d(0,0), new Vector2d(0,0));

        Vector2d robotToCamera =  new Vector2d(-9.37 / 25.4, 488.84 / 25.4);
        Vector2d robotToGripper =  new Vector2d(-14.75 / 25.4, 411.71 / 25.4);
        Vector2d cameraToGripper = robotToCamera.minus(robotToGripper);
        System.out.println(cameraToGripper);

        double NORTH = Math.toRadians(90);
        double EAST = Math.toRadians(0);
        double SOUTH = Math.toRadians(270);
        double WEST = Math.toRadians(180);
        double BASKET = Math.toRadians(225);

        Pose2d initialPose = new Pose2d(-9, -63, NORTH);
        Pose2d chamber = new Pose2d(-10, -33, NORTH);
        Pose2d chamberScore = new Pose2d(-10, -33, NORTH);
        Pose2d sample1 = new Pose2d(-47.5, -45, NORTH);
        Pose2d sample2 = new Pose2d(-57.5, -42, NORTH);
        Pose2d sample3 = new Pose2d(-52, -25, WEST);
        Pose2d basket1 = new Pose2d(-53, -53, BASKET);
        Pose2d basket2 = new Pose2d(-53, -53, BASKET);
        Pose2d basket3 = new Pose2d(-53, -53, BASKET);
        Pose2d park = new Pose2d(-24, -12, EAST);

        Pose2d sample1Pose = new Pose2d(-47.5,-45, WEST);

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                //drive to chamber
                //TODO: Test if this can be done while raising lift?
                .splineToSplineHeading(chamber,NORTH)
                // pause to simulate lifting to chamber
                .waitSeconds(0.66)
                // pause to simulate arm moving to chamber
                .waitSeconds(0.25)
                // pause to simulate lift lowering to clipped position
                .waitSeconds(0.1)
                // pause to simulate gripper opening
                .waitSeconds(0.1)
                // pause to simulate lowering to scanning height
                .waitSeconds(0.66)

                //drive to first sample
                .setReversed(true)
                //.strafeTo(new Vector2d(-47,-40))
                //.splineToConstantHeading(new Vector2d(-47,-40), WEST)
                .splineToLinearHeading(sample1, WEST)
                // pause to simulate scanning
                .waitSeconds(0.1)

                //drive to baskets
                .setReversed(true)
                .splineToSplineHeading(basket1,SOUTH)
                // pause to simulate lift to high basket
                .waitSeconds(1)
                // pause to simulate arm to scoring pos
                .waitSeconds(0.5)
                // pause to simulate gripper opening
                .waitSeconds(0.1)
                // pause to simulate lift to scanning height and arm to horizontal
                .waitSeconds(1)

                //drive to second sample
                .setReversed(true)
                .splineToSplineHeading(sample2, NORTH)
                // pause to simulate scanning and moving to sample
                .waitSeconds(0.5)
                // pause to simulate gripper closing
                .waitSeconds(0.1)
                // pause to simulate arm to vertical
                .waitSeconds(1)
                //drive to baskets
                .setReversed(true)
                .splineToSplineHeading(basket2, SOUTH)
                // pause to simulate lift to high basket
                .waitSeconds(1)
                // pause to simulate arm to scoring pos
                .waitSeconds(0.5)
                // pause to simulate gripper opening
                .waitSeconds(0.1)
                // pause to simulate lift to scanning height and arm to horizontal
                .waitSeconds(1)

                //drive to third sample
                .setReversed(true)
                .splineToSplineHeading(sample3, NORTH)
                // pause to simulate scanning and moving to sample
                .waitSeconds(0.5)
                // pause to simulate gripper closing
                .waitSeconds(0.1)
                // pause to simulate arm to vertical
                .waitSeconds(1)
                //drive to baskets
                .setReversed(true)
                .splineToSplineHeading(basket3 ,SOUTH)
                // pause to simulate lift to high basket
                .waitSeconds(1)
                // pause to simulate arm to scoring pos
                .waitSeconds(0.5)
                // pause to simulate gripper opening
                .waitSeconds(0.1)
                // pause to simulate lift to chamber height and arm to vertical
                .waitSeconds(1)

                //drive to park - need to simultaneously move lift to ~150mm
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-24,-12, EAST), EAST)
                // pause to simulate touching rung (lower arm to rest on rung)
                .waitSeconds(1)

                .build());
/*
        Vector2d samplePosition = sample1Pose.position.plus(Rotation2d.exp(sample1Pose.heading.toDouble()-toRadians(90)).times(cameraToGripper));
        Vector2d returnPosition = sample1Pose.position;

        myBot2.runAction(myBot.getDrive().actionBuilder(sample1Pose)

                .strafeTo(samplePosition)
                // pause to simulate gripper closing
                .waitSeconds(0.1)
                .strafeTo(returnPosition)
                // pause to simulate arm to vertical
                .waitSeconds(1)
                .build());*/

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                //.addEntity(myBot2)
                .addEntity(cameraCentre)
                .addEntity(gripperCentre)
                .start();
    }
}