package com.example.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

//import org.firstinspires.ftc.teamcode.autonomous.autos.FieldConstants;

import org.jetbrains.annotations.NotNull;

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
//            .setTangent(Math.toRadians(0))
            .strafeToLinearHeading(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE)
            .build();

        Action gpp = drive.actionBuilder(new Pose2d(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE))
            .setTangent(0)
            .splineToSplineHeading(new Pose2d(new Vector2d(FCV2.GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST+10), FCV2.BLUE_ARTIFACT_ANGLE), Math.toRadians(90))
            .build();

        Action gpp_return = drive.actionBuilder(new Pose2d(new Vector2d(FCV2.GPP_BLUE_ARTIFACT.x, FCV2.GPP_BLUE_ARTIFACT.y+FCV2.ARTIFACT_DIST+10), FCV2.BLUE_ARTIFACT_ANGLE))
            .strafeToLinearHeading(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE)
            .build();

        Action human1 = drive.actionBuilder(new Pose2d(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE))
//            .setTangent(0)
//
//            .splineToLinearHeading(new Pose2d(FCV2.HP_BLUE_ARTIFACT, Math.PI), Math.PI, new VelConstraint() {
//                @Override
//                public double maxRobotVel(@NotNull Pose2dDual<Arclength> pose2dDual, @NotNull PosePath posePath, double v) {
//                    return 20;
//                }
//            })
            .strafeToLinearHeading(new Vector2d(FCV2.HP_BLUE_ARTIFACT.x, FCV2.HP_BLUE_ARTIFACT.y - 10), FCV2.BLUE_ARTIFACT_ANGLE)
            .strafeToLinearHeading(new Vector2d(FCV2.HP_BLUE_ARTIFACT.x, FCV2.HP_BLUE_ARTIFACT.y), FCV2.BLUE_ARTIFACT_ANGLE)
            .build();
//
        Action human1_return = drive.actionBuilder(new Pose2d(new Vector2d(FCV2.HP_BLUE_ARTIFACT.x, FCV2.HP_BLUE_ARTIFACT.y), FCV2.BLUE_ARTIFACT_ANGLE))
//            .setTangent(Math.toRadians(0))
            .strafeToSplineHeading(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE)
            .build();
//
        Action human2 = drive.actionBuilder(new Pose2d(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE))
            .turnTo(FCV2.BLUE_ARTIFACT_ANGLE)
            .strafeToConstantHeading(FCV2.HP_BLUE_ARTIFACT)
            .build();

        Action human2_return = drive.actionBuilder(new Pose2d(FCV2.HP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE))
            .strafeToLinearHeading(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE)
            .build();

        Action human3 = drive.actionBuilder(new Pose2d(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE))
            .turnTo(FCV2.BLUE_ARTIFACT_ANGLE)
            .strafeToConstantHeading(FCV2.HP_BLUE_ARTIFACT)
            .build();

        Action human3_return = drive.actionBuilder(new Pose2d(FCV2.HP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE))
            .strafeToLinearHeading(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE)
            .build();

        d.runAction(
            new SequentialAction(
                preload,
                gpp,
                gpp_return,
                human1,
                human1_return,
                human2,
                human2_return,
                human3,
                human3_return
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