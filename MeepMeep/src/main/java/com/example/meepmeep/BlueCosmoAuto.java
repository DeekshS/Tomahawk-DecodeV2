package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class BlueCosmoAuto {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
            .setConstraints(90, 90, 2.5, 3, 18)
            .setDimensions(16.53, 18)
            .build();

        Action preload = drive.getDrive().actionBuilder(FCV2.BLUE_FAR_START)
                .strafeToLinearHeading(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE)

                .build();

        Action gpp = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_FAR_SHOOT.x, FCV2.BLUE_FAR_SHOOT.y, FCV2.BLUE_FAR_ANGLE))
            .strafeToLinearHeading(FCV2.GPP_BLUE_ARTIFACT, Math.toRadians(90))
            .setTangent(Math.toRadians(90))
            .lineToY(FCV2.GPP_BLUE_ARTIFACT.y + FCV2.ARTIFACT_DIST)
            .build();

        Action ppg_return = drive.getDrive().actionBuilder(new Pose2d(FCV2.GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST, FCV2.BLUE_ARTIFACT_ANGLE))
            .strafeToLinearHeading(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE)
            .build();

        Action intake1 = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE))
                .strafeToLinearHeading(FCV2.BLUE_GATE_INTAKE, 0)
                .build();

        Action gate_return = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_GATE_INTAKE, 0))
                .strafeToLinearHeading(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE)
                .build();

        Action human1 = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_FAR_START.component1(), FCV2.BLUE_ARTIFACT_ANGLE))
            .setTangent(0)
            .splineToSplineHeading(new Pose2d(FCV2.HP_BLUE_ARTIFACT, Math.PI), Math.PI)
            .build();

        Action human1_return = drive.getDrive().actionBuilder(new Pose2d(FCV2.HP_BLUE_ARTIFACT, Math.PI))
            .setTangent(Math.toRadians(0))
            .splineToLinearHeading(FCV2.BLUE_FAR_START, Math.toRadians(180))
            .build();

        Action human2 = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_FAR_START.component1(), FCV2.BLUE_ARTIFACT_ANGLE))
            .strafeToLinearHeading(FCV2.HP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE)
            .build();

        Action human2_return = drive.getDrive().actionBuilder(new Pose2d(FCV2.HP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE))
            .strafeToLinearHeading(FCV2.BLUE_FAR_START.component1(), FCV2.BLUE_ARTIFACT_ANGLE)
            .build();

        drive.runAction(
            new SequentialAction(
                    preload,
                    gpp,
                    ppg_return,
                    intake1,
                    gate_return,
                    intake1,
                    gate_return
//                human1,
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
            .addEntity(drive)
            .start();


    }
}