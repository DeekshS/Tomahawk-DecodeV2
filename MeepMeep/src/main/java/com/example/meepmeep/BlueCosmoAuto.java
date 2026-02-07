package com.example.meepmeep;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
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


        Action hp = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE))
            .setTangent(Math.toRadians(100))
            .splineToSplineHeading(new Pose2d(-60, 59, Math.toRadians(90)), Math.toRadians(100))
            .build();


        Action hp_shoot = drive.getDrive().actionBuilder(new Pose2d(new Vector2d( -60, 59), Math.toRadians(90)))
            .strafeToLinearHeading(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE)
            .build();


        Action hp_up = drive.getDrive().actionBuilder(new Pose2d(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE))
            .setTangent(Math.toRadians(30))
            .splineToSplineHeading(new Pose2d(-47, 59, Math.toRadians(90)), Math.toRadians(90))


            .build();


        Action hp_up_shoot = drive.getDrive().actionBuilder(new Pose2d(-47, 59, Math.toRadians(90)))
            .strafeToLinearHeading(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE)


            .build();




        drive.runAction(
            new SequentialAction(
                preload,
                hp,
                hp_shoot,
                hp_up,
                hp_up_shoot
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

