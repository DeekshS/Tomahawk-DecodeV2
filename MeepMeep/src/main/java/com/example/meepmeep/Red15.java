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
            .strafeToLinearHeading(new Vector2d(FCV2.PPG_RED_ARTIFACT.x, FCV2.PPG_RED_ARTIFACT.y), FCV2.RED_ARTIFACT_ANGLE)
            .strafeToConstantHeading(new Vector2d(FCV2.PPG_RED_ARTIFACT.x, FCV2.PPG_RED_ARTIFACT.y - FCV2.ARTIFACT_DIST - 1))
            .strafeToLinearHeading(FCV2.RED_GATE, 0)
            .build();


        Action artifact1_return = drive.actionBuilder(new Pose2d(FCV2.RED_GATE.x, FCV2.RED_GATE.y, 0))
            .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE)
            .build();


        Action artifact2 = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .strafeToLinearHeading(FCV2.PGP_RED_ARTIFACT, FCV2.RED_ARTIFACT_ANGLE)

            .setTangent(FCV2.RED_ARTIFACT_ANGLE)

            .strafeToConstantHeading(new Vector2d(FCV2.PGP_RED_ARTIFACT.x, FCV2.PGP_RED_ARTIFACT.y - FCV2.ARTIFACT_DIST- 2))
//            .setReversed(true)
//            .splineToLinearHeading(new Pose2d(FCV2.PGP_RED_ARTIFACT.x, FCV2.PGP_RED_ARTIFACT.y+FCV2.ARTIFACT_DIST, FCV2.RED_ARTIFACT_ANGLE), -Math.PI/2.2)

            .build();

        Action artifact2_return = drive.actionBuilder(new Pose2d(FCV2.PGP_RED_ARTIFACT.x, FCV2.PGP_RED_ARTIFACT.y - FCV2.ARTIFACT_DIST - 2, FCV2.RED_ARTIFACT_ANGLE))
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE), Math.PI/8)

            .build();



        Action artifact3 = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .strafeToLinearHeading(FCV2.GPP_RED_ARTIFACT_CLOSE, FCV2.RED_ARTIFACT_ANGLE)
            .strafeToConstantHeading(new Vector2d(FCV2.GPP_RED_ARTIFACT_CLOSE.x, FCV2.GPP_RED_ARTIFACT_CLOSE.y - FCV2.ARTIFACT_DIST - 2))
//                .strafeToLinearHeading(FCV2.RED_GATE, 0)
            .build();

        Action artifact3_return = drive.actionBuilder(new Pose2d(FCV2.GPP_RED_ARTIFACT_CLOSE.x, FCV2.GPP_RED_ARTIFACT_CLOSE.y - FCV2.ARTIFACT_DIST, FCV2.RED_ARTIFACT_ANGLE))

            //                .setReversed(true)
            .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE)

            .build();

        Action park = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .strafeTo(new Vector2d(FCV2.PGP_RED_ARTIFACT.x, FCV2.PGP_RED_ARTIFACT.y - 5))
            .build();

        Action hp = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
            .strafeToLinearHeading(new Vector2d(FCV2.HP_RED_ARTIFACT.x + 14, FCV2.HP_RED_ARTIFACT.y), Math.toRadians(-180))
            .strafeTo(new Vector2d(FCV2.HP_RED_ARTIFACT.x, FCV2.HP_RED_ARTIFACT.y))
            .build();

        Action hp_return = drive.actionBuilder(new Pose2d(FCV2.HP_RED_ARTIFACT.x, FCV2.HP_RED_ARTIFACT.y, Math.toRadians(-180)))
//                .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE)
            .setTangent(Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE), Math.toRadians(-180))
            .build();

        Action hp2 = drive.actionBuilder(new Pose2d(FCV2.RED_CLOSE_SHOOT.x, FCV2.RED_CLOSE_SHOOT.y, FCV2.RED_CLOSE_ANGLE))
//            .strafeToLinearHeading(FCV2.HP_RED_ARTIFACT, Math.toRadians(180))
            .strafeToLinearHeading(new Vector2d(FCV2.HP_RED_ARTIFACT.x - 10, FCV2.HP_RED_ARTIFACT.y), Math.toRadians(180))
            .strafeTo(new Vector2d(FCV2.HP_RED_ARTIFACT.x, FCV2.HP_RED_ARTIFACT.y))
            .build();

        Action hp_return2 = drive.actionBuilder(new Pose2d(FCV2.HP_RED_ARTIFACT.x, FCV2.HP_RED_ARTIFACT.y, Math.toRadians(180)))
            .strafeToLinearHeading(FCV2.RED_CLOSE_SHOOT, FCV2.RED_CLOSE_ANGLE)
            .build();

        bot.runAction(
            new SequentialAction(
                preload,
                artifact2,
                artifact2_return,
                artifact1,
                artifact1_return,
                artifact3,
                artifact3_return,
                hp,
                hp_return
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