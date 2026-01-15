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


public class Red15 {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
            .setConstraints(90, 90, 2.5, 3, 18)
            .setDimensions(16.53, 18)
            .build();
        DriveShim drive = bot.getDrive();

        Action preload = drive.actionBuilder(FCV2.RED_CLOSE_START)
            .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE)
            .build();

        Action artifact1 = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .setTangent(Math.toRadians(-180))
            .splineToSplineHeading(new Pose2d(FCV2.PGP_RED_ARTIFACT.x, FCV2.PGP_RED_ARTIFACT.y, FCV2.RED_ARTIFACT_ANGLE), Math.toRadians(-90))
            .build();

        Action artifact1_return = drive.actionBuilder(new Pose2d(FCV2.PGP_RED_ARTIFACT, FCV2.RED_ARTIFACT_ANGLE))

            .setTangent(Math.toRadians(-285))
            .splineToLinearHeading(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE), Math.toRadians(-45))
            .build();


        Action artifact2 = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .strafeToLinearHeading(FCV2.PPG_RED_ARTIFACT, FCV2.RED_ARTIFACT_ANGLE)
            .build();

        Action artifact2_return = drive.actionBuilder(new Pose2d(FCV2.PPG_RED_ARTIFACT.x, FCV2.PPG_RED_ARTIFACT.y, FCV2.RED_ARTIFACT_ANGLE))

            .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE)
            .build();

        Action gate_score = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .setTangent(Math.toRadians(-180))
            .splineToLinearHeading(new Pose2d(FCV2.RED_GATE.x - 16, FCV2.RED_GATE.y-2, Math.toRadians(-65)), Math.toRadians(-90))            .build();

        Action gate_return = drive.actionBuilder(new Pose2d(FCV2.RED_GATE.x - 16, FCV2.RED_GATE.y-2, Math.toRadians(-65)))
            .setTangent(Math.toRadians(-180))
            .splineToLinearHeading(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE), Math.toRadians(-90))
            .build();

        Action gate_score2 = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .setTangent(Math.toRadians(-180))
            .splineToLinearHeading(new Pose2d(FCV2.RED_GATE.x - 16, FCV2.RED_GATE.y-2, Math.toRadians(-65)), Math.toRadians(-90))
            .build();

        Action gate_return2 = drive.actionBuilder(new Pose2d(FCV2.RED_GATE.x - 16, FCV2.RED_GATE.y-2, Math.toRadians(65)))
            .setTangent(Math.toRadians(-180))
            .splineToLinearHeading(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE), Math.toRadians(90))
            .build();


        Action artifact3 = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .setTangent(Math.toRadians(-190))
            .splineToSplineHeading(new Pose2d(FCV2.GPP_RED_ARTIFACT.x, FCV2.GPP_RED_ARTIFACT.y, FCV2.RED_CLOSE_ANGLE), Math.toRadians(-90))
            .build();

        Action artifact3_return = drive.actionBuilder(new Pose2d(FCV2.GPP_RED_ARTIFACT.x, FCV2.GPP_RED_ARTIFACT.y, FCV2.RED_ARTIFACT_ANGLE))
            .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE)
            .build();

        bot.runAction(
            new SequentialAction(
                preload,
                artifact1,

                new ParallelAction(
                    artifact1_return
                ),

                gate_score,
                gate_return,

                gate_score,
                gate_return,

                gate_score,
                gate_return,

                // ARTIFACT 2

                new ParallelAction(
                    artifact2
                ),

                new ParallelAction(
                    artifact2_return
                ),
                // ARTIFACT 3

                new ParallelAction(
                    artifact3
                ),

                new ParallelAction(
                    artifact3_return
                )
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
            .addEntity(bot)
            .start();


    }
}