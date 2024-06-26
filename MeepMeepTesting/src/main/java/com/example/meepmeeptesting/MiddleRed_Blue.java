package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MiddleRed_Blue {
    static int randomization;
    static int MAX = 3;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);
        Pose2d rightPoseSpike = new Pose2d(-46, 34, Math.toRadians(90));
        Pose2d leftPoseSpike = new Pose2d(-34, 32, Math.toRadians(180));
        // Use reset pose 2 times
        Pose2d centerPoseSpike = new Pose2d(-34, 32 , Math.toRadians(90));
        // Use reset pose 2 times
        Pose2d reset = new Pose2d(-40, 58, Math.toRadians(180));
        Pose2d backdropPose = new Pose2d(47, 60, Math.toRadians(180));
        Pose2d rightScoringPose = new Pose2d(49, 28, Math.toRadians(180));
        Pose2d midScoringPose = new Pose2d(49,34, Math.toRadians(180));
        Pose2d leftScoringPose = new Pose2d(49, 40, Math.toRadians(180));

        RoadRunnerBotEntity blueAllianceRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme((new ColorSchemeBlueDark()))
                .setDimensions(16, 16)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(270), 11.15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-40, 62, Math.toRadians(90)))
                                .lineToLinearHeading(centerPoseSpike)

                                // Grab 1 Pixel
                                .lineToLinearHeading(new Pose2d(-58, 35, Math.toRadians(180)))

                                //Yellow Pixel + 1
                                .lineToLinearHeading(new Pose2d(51, 35, Math.toRadians(180)))

                                .waitSeconds(1)

                                // 1st Cycle
                                .lineToLinearHeading(new Pose2d(-58, 35, Math.toRadians(180)))

                                .waitSeconds(1)

                                //Cycle End
                                .lineToLinearHeading(new Pose2d(51, 35, Math.toRadians(180)))

                                .waitSeconds(1)

                                // 1st Cycle
                                .lineToLinearHeading(new Pose2d(-58, 35, Math.toRadians(180)))

                                .waitSeconds(1)

                                //Cycle End
                                .lineToLinearHeading(new Pose2d(51, 35, Math.toRadians(180)))

                                .lineToConstantHeading(new Vector2d(-35, 35))
                                .splineToConstantHeading(new Vector2d(-57, 23), Math.toRadians(180))

                                .strafeRight(5)
                                .splineToConstantHeading(new Vector2d(-35, 35), Math.toRadians(0))

                                .lineToLinearHeading(new Pose2d(51, 35, Math.toRadians(180)))

                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueAllianceRight)
                .start();
    }
};
