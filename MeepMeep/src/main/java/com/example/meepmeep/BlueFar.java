package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//import org.firstinspires.ftc.teamcode.autonomous.autos.FieldConstants;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class BlueFar {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity d = new DefaultBotBuilder(meepMeep)
            .setConstraints(90, 90, 2.5, 3, 18)
            .setDimensions(16.53, 18)
            .build();

        DriveShim drive = d.getDrive();

        Action preload = drive.actionBuilder(FCV2.BLUE_FAR_START)
            .setTangent(Math.toRadians(0))
            .strafeToLinearHeading(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE)
            .build();

//        Action human1 = drive.actionBuilder(new Pose2d(FCV2.BLUE_FAR_SHOOT.component1(), FCV2.BLUE_FAR_ANGLE))
//            .setTangent(0)
//            .splineToSplineHeading(new Pose2d(FCV2.HP_BLUE_ARTIFACT, Math.PI), Math.PI)
//            .build();
//
//        Action human1_return = drive.actionBuilder(new Pose2d(FCV2.HP_BLUE_ARTIFACT, Math.PI))
//            .setTangent(Math.toRadians(0))
//            .splineToLinearHeading(FCV2.BLUE_FAR_START, Math.toRadians(180))
//            .build();
//
//        Action human2 = drive.actionBuilder(new Pose2d(FCV2.BLUE_FAR_START.component1(), FCV2.BLUE_ARTIFACT_ANGLE))
//            .strafeToLinearHeading(FCV2.HP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE)
//            .build();
//
//        Action human2_return = drive.actionBuilder(new Pose2d(FCV2.HP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE))
//            .strafeToLinearHeading(FCV2.BLUE_FAR_START.component1(), FCV2.BLUE_ARTIFACT_ANGLE)
//            .build();

        d.runAction(
            new SequentialAction(
                preload

//                human1_return,
//                human2,
//                human2_return
            )

        );

        BufferedImage bg = null;
        try {
            bg = ImageIO.read(new File("MeepMeep/src/main/java/com/example/meepmeep/DECODE.png"));
        } catch (IOException e) {
            e.printStackTrace();
        }
        meepMeep.setBackground(bg)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(d)
            .start();


    }
}