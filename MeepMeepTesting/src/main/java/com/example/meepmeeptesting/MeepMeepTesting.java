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
                .setConstraints(60, 45, Math.toRadians(180), Math.toRadians(180), 13.28)
                .setDimensions(18,18)
                .build();

         /*myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-48, -36, 0))
                .lineToX(36)
                .turn(Math.toRadians(90))
                .lineToY(36)
                .turn(Math.toRadians(90))
                .lineToX(-36)
                .turn(Math.toRadians(90))
                .lineToY(-36)
                .turn(Math.toRadians(90))
                .build());*/

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, -36, 0))
                .splineToSplineHeading(new Pose2d(36,-36, Math.toRadians(90)),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(36,36, Math.toRadians(180)),Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-36,36, Math.toRadians(270)),Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-36,-36, 0),0)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}