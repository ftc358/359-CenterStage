package com.example.eepeep;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class EepEep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, toRadians(90)))
                                //.splineToLinearHeading(new Pose2d(49, -25,Math.toRadians(0)),Math.toRadians(178))
                                //new SequentialAction()
                                //
                                //.strafeTo(new Vector2d(-60, -36))
                                .splineToLinearHeading(new Pose2d(7-48, -32, toRadians(0)), 90)
                                .strafeTo(new Vector2d(-38,0))
                                .splineToLinearHeading(new Pose2d(49, -23.5, toRadians(178)), toRadians(0))
                                .strafeTo(new Vector2d(53,-55))

                                //.splineToLinearHeading(new Pose2d(-39, -30, Math.toRadians(0)), 0)

                                //.strafeTo(new Vector2d(-24,-37))
                                //.strafeTo(new Vector2d(-38,0))
                                //.splineToLinearHeading(new Pose2d(49, 27,Math.toRadians(180)), Math.toRadians(0))
                               // .strafeTo(new Vector2d(49,-37))
                               //.splineToLinearHeading(new Pose2d(49, -37, toRadians(178)), toRadians(0))


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}