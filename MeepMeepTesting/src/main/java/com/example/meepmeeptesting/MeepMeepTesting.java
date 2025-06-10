package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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

        double NORTH = Math.toRadians(90);
        double EAST = Math.toRadians(0);
        double SOUTH = Math.toRadians(270);
        double WEST = Math.toRadians(180);
        double BASKET = Math.toRadians(225);


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-9, -63, NORTH))
                //drive to chamber
                //TODO: Test if this can be done while raising lift?
                .splineToSplineHeading(new Pose2d(-10,-33, NORTH),NORTH)
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
                .splineToLinearHeading(new Pose2d(-47,-40, NORTH), WEST)
                // pause to simulate scanning and moving to sample
                .waitSeconds(0.5)
                // pause to simulate gripper closing
                .waitSeconds(0.1)
                // pause to simulate arm to vertical
                .waitSeconds(1)
                //drive to baskets
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-56,-56, BASKET),SOUTH)
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
                .splineToSplineHeading(new Pose2d(-58,-40, NORTH), NORTH)
                // pause to simulate scanning and moving to sample
                .waitSeconds(0.5)
                // pause to simulate gripper closing
                .waitSeconds(0.1)
                // pause to simulate arm to vertical
                .waitSeconds(1)
                //drive to baskets
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-56,-56, BASKET),SOUTH)
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
                .splineToSplineHeading(new Pose2d(-58,-26, WEST), NORTH)
                // pause to simulate scanning and moving to sample
                .waitSeconds(0.5)
                // pause to simulate gripper closing
                .waitSeconds(0.1)
                // pause to simulate arm to vertical
                .waitSeconds(1)
                //drive to baskets
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-56,-56, BASKET),SOUTH)
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

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}