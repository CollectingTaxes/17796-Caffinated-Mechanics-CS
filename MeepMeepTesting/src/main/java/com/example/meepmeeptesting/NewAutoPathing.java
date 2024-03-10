package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Robot;

public class NewAutoPathing {
    public static void main(String[] args) {
        Pose2d closeRightSpike = new Pose2d(15, -39, Math.toRadians(45));
        Pose2d closeLeftSpike = new Pose2d(6, -39, Math.toRadians(180 - 45));

        Pose2d closeRightScore = new Pose2d(49, -44, Math.toRadians(180));

        MeepMeep meepMeep = new MeepMeep(650);
        RoadRunnerBotEntity Robot = new DefaultBotBuilder(meepMeep)
                .setColorScheme((new ColorSchemeBlueLight()))
                .setDimensions(14, 16)
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(270), 11.15)
                .followTrajectorySequence(driveShim ->
                        driveShim.trajectorySequenceBuilder(new Pose2d(7, -63, Math.toRadians(90)))
                                .lineToLinearHeading(closeRightSpike)
                                .lineToLinearHeading(closeRightScore)
                                .waitSeconds(1)
                                //Cycle 1 start
                                .lineToLinearHeading(new Pose2d(30, -25, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-59, -11), Math.toRadians(180))
                                .waitSeconds(1)
                                //Cycle 1 end
                                .lineToLinearHeading(new Pose2d(0, -14, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(49, -43), Math.toRadians(270))
                                .waitSeconds(1)
                                //Cycle 2 start
                                .lineToLinearHeading(new Pose2d(30, -25, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-59, -11), Math.toRadians(180))
                                //Cycle 2 end
                                .lineToLinearHeading(new Pose2d(0, -14, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(49, -43), Math.toRadians(270))
                                .build()

                );
        RoadRunnerBotEntity Robot2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                .setDimensions(13, 16)
                .setConstraints(45,45, Math.toRadians(180), Math.toRadians(270), 11.15)
                .followTrajectorySequence(driveShim ->
                        driveShim.trajectorySequenceBuilder(new Pose2d(-30, 63, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-49, 50, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(-30, 40, Math.toRadians(315)))
                                .lineToLinearHeading(new Pose2d(-60,23, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-41, 18, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(49, 43), Math.toRadians(90))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Robot2)
                .start();
    }
}