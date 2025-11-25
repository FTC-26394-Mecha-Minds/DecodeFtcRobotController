package com.example.meepmeepvisualizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepVisualizer {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(75, 75, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-62, 36, 0))
                .splineToLinearHeading(new Pose2d(-22, 16, Math.toRadians(310)),0)
                .waitSeconds(4.5)
                .splineToLinearHeading(new Pose2d(-12, 34, Math.toRadians(90)), 0)
                .waitSeconds(0.5)
                .lineToYConstantHeading(38)
                .waitSeconds(0.5)
                .lineToYConstantHeading(42)
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(-22, 16, Math.toRadians(310)),0)
                .waitSeconds(4.5)
                .splineToLinearHeading(new Pose2d(12, 34, Math.toRadians(90)), 0)
                .waitSeconds(0.5)
                .lineToYConstantHeading(38)
                .waitSeconds(0.5)
                .lineToYConstantHeading(42)
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(-22, 16, Math.toRadians(310)),0)
                .waitSeconds(4.5)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}