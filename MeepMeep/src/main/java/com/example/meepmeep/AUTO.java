package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class AUTO {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 70, 2.5, 3, 18)
                .setDimensions(16.53, 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(1, 1, 1))
                .build()
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
                .addEntity(myBot)
                .start();


    }
}