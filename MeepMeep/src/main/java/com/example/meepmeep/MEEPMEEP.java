package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MEEPMEEP {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(73.17330064499293, 73.17330064499293, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(drive ->
                        //BlueFar Left
                        // y up and down, x left and right
                        // -60, 10 (POSITION FOR PIXELS)
                  //    drive.trajectorySequenceBuilder(new Pose2d(-35, 61, Math.toRadians(90)))
                   //           .splineToLinearHeading(new Pose2d(-33,34, Math.toRadians(180)), Math.toRadians(90))
                   //           .forward(10)
                   //           .lineTo(new Vector2d(-39, 10))
                   //           .back(70)
                   //           .lineToConstantHeading(new Vector2d(51, 46))
                        //BlueFar Center
                        //   .splineToSplineHeading(new Pose2d(-35,32, Math.toRadians(115)), Math.toRadians(90))
                        //   .splineToLinearHeading(new Pose2d(-50,42, Math.toRadians(115)), Math.toRadians(270))
                        //   .lineTo(new Vector2d(-48, 15))
                        //   .splineToSplineHeading(new Pose2d(26, 11, Math.toRadians(180)), Math.toRadians(0))
                        //   .splineToSplineHeading(new Pose2d(52, 35, Math.toRadians(180)), Math.toRadians(40))
                        //BlueFar Right
                       //        .lineToSplineHeading(new Pose2d(-47,35, Math.toRadians(80)))
                       //        .lineToSplineHeading(new Pose2d(-34,50, Math.toRadians(90)))
                       //        .forward(-30)
                       //        .splineToSplineHeading(new Pose2d(20,10, Math.toRadians(180)),Math.toRadians(15))
                       //        .lineTo(new Vector2d(52, 22))

                        drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(270)))
                                .splineToLinearHeading(new Pose2d(-33,-34, Math.toRadians(180)), Math.toRadians(90))
                                .forward(10)
                                .lineTo(new Vector2d(-39, -10))
                                .back(70)
                                .lineToConstantHeading(new Vector2d(55, -46))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
             //   .addEntity(myBot)
                .start();
    }
}