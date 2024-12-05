package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepTurbo {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(500, 60, Math.toRadians(1000), Math.toRadians(1000), 339/2.54)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36.35, -62.84, Math.toRadians(47.22)))
//                                .forward(50)
//                                .turn(Math.toRadians(90))
//                                .forward(50)
//                                .addDisplacementMarker(() -> {
//                                    /* Everything in the marker callback should be commented out */
//
//                                    // bot.shooter.shoot()
//                                    // bot.wobbleArm.lower()
//                                })
//                                .turn(Math.toRadians(90))
//                                .splineTo(new Vector2d(50, 15), 0)
//                                .turn(Math.toRadians(90))
//                                .build()
                        //TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(36.35, -62.84, Math.toRadians(47.22)))
                        .splineTo(new Vector2d(60.65, -35.75), Math.toRadians(90.00))
                        .splineTo(new Vector2d(61.24, 37.15), Math.toRadians(90.00))
                        .splineTo(new Vector2d(35.95, 60.05), Math.toRadians(180.00))
                        .build());






        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

