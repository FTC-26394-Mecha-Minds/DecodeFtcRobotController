package com.example.meepmeepvisualizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepVisualizer {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity redclassifierBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(75, 75, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redclassifierBot.runAction(redclassifierBot.getDrive().actionBuilder(new Pose2d(-62, 36, 0))
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

        RoadRunnerBotEntity blueclassifierBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(75, 75, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        blueclassifierBot.runAction(blueclassifierBot.getDrive().actionBuilder(new Pose2d(-62, -36, 0))
                .splineToLinearHeading(new Pose2d(-22, -16, Math.toRadians(50)),0)
                .waitSeconds(4.5)
                .splineToLinearHeading(new Pose2d(-12, -34, Math.toRadians(270)), 0)
                .waitSeconds(0.5)
                .lineToYConstantHeading(-38)
                .waitSeconds(0.5)
                .lineToYConstantHeading(-42)
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(-22, -16, Math.toRadians(50)),0)
                .waitSeconds(4.5)
                .splineToLinearHeading(new Pose2d(12, -34, Math.toRadians(270)), 0)
                .waitSeconds(0.5)
                .lineToYConstantHeading(-38)
                .waitSeconds(0.5)
                .lineToYConstantHeading(-42)
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(-22, -16, Math.toRadians(50)),0)
                .waitSeconds(4.5)
                .build());

        RoadRunnerBotEntity redfarBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(75, 75, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        redfarBot.runAction(redfarBot.getDrive().actionBuilder(new Pose2d(60, 16, 0))
                .splineToLinearHeading(new Pose2d(46, 8, Math.toRadians(330)),0)
                .waitSeconds(4.5)
                .splineToLinearHeading(new Pose2d(36, 34, Math.toRadians(90)), 0)
                .waitSeconds(0.5)
                .lineToYConstantHeading(38)
                .waitSeconds(0.5)
                .lineToYConstantHeading(46)
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(46, 8, Math.toRadians(330)),0)
                .waitSeconds(4.5)
                .splineToLinearHeading(new Pose2d(58, 58, Math.toRadians(65)), 0)
                .waitSeconds(4.5)
                .splineToLinearHeading(new Pose2d(46, 8, Math.toRadians(330)),0)
                .waitSeconds(4.5)
                .build());
        RoadRunnerBotEntity bluefarBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(75, 75, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        bluefarBot.runAction(bluefarBot.getDrive().actionBuilder(new Pose2d(60, -16, 0))
                .splineToLinearHeading(new Pose2d(46, -8, Math.toRadians(30)),0)
                .waitSeconds(4.5)
                .splineToLinearHeading(new Pose2d(36, -34, Math.toRadians(270)), 0)
                .waitSeconds(0.5)
                .lineToYConstantHeading(-38)
                .waitSeconds(0.5)
                .lineToYConstantHeading(-46)
                .waitSeconds(0.5)
                .splineToLinearHeading(new Pose2d(46, -8, Math.toRadians(30)),0)
                .waitSeconds(4.5)
                .splineToLinearHeading(new Pose2d(58, -58, Math.toRadians(295)), 0)
                .waitSeconds(4.5)
                .splineToLinearHeading(new Pose2d(46, -8, Math.toRadians(30)),0)
                .waitSeconds(4.5)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueclassifierBot)
                .addEntity(redclassifierBot)
                .addEntity(redfarBot)
                .addEntity(bluefarBot)
                .start();
    }
}