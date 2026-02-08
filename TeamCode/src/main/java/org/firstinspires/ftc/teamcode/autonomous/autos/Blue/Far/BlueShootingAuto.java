
package org.firstinspires.ftc.teamcode.autonomous.autos.Blue.Far;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.autos.BotActions;
import org.firstinspires.ftc.teamcode.autonomous.autos.FCV2;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.PoseStorage;

@Autonomous
@Config
public class BlueShootingAuto extends LinearOpMode implements FCV2 {

    public static double INTAKE_WAIT_TIME = 1.4;
    public static double SHOOTER_TIME = 1.25;

    public static int ARTIFACT_SHOOT_VEL = 2000;
    public static double HOOD_POS = 0.2;


    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);

        MecanumDrive drive = new MecanumDrive(hardwareMap, FCV2.BLUE_FAR_START);

        Action preload = drive.actionBuilder(FCV2.BLUE_FAR_START)
            .setTangent(Math.toRadians(0))
            .strafeToLinearHeading(FCV2.BLUE_FAR_SHOOT, FCV2.BLUE_FAR_ANGLE)
            .build();

        Action human1 = drive.actionBuilder(new Pose2d(FCV2.BLUE_FAR_START.component1(), FCV2.BLUE_ARTIFACT_ANGLE))
            .setTangent(0)
            .splineToSplineHeading(new Pose2d(FCV2.HP_BLUE_ARTIFACT, Math.PI), Math.PI)
            .build();

        Action human1_return = drive.actionBuilder(new Pose2d(FCV2.HP_BLUE_ARTIFACT, Math.PI))
            .setTangent(Math.toRadians(0))
            .splineToLinearHeading(FCV2.BLUE_FAR_START, Math.toRadians(180))
            .build();

        Action human2 = drive.actionBuilder(new Pose2d(FCV2.BLUE_FAR_START.component1(), FCV2.BLUE_ARTIFACT_ANGLE))
            .strafeToLinearHeading(FCV2.HP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE)
            .build();

        Action human2_return = drive.actionBuilder(new Pose2d(FCV2.HP_BLUE_ARTIFACT, FCV2.BLUE_ARTIFACT_ANGLE))
            .strafeToLinearHeading(FCV2.BLUE_FAR_START.component1(), FCV2.BLUE_ARTIFACT_ANGLE)
            .build();




        waitForStart();
        if (isStopRequested()) return;


        Actions.runBlocking(
            new ParallelAction(
                robot.outtake.hoodAction(HOOD_POS, 29.9),
                robot.outtake.shootVelocityTimeAction(ARTIFACT_SHOOT_VEL, 29.9),
                robot.intake.intakeTimeAction(29.9),
                robot.turret.alignAction(-40, 29.9),
                new SequentialAction(
                    new ParallelAction(
                        preload,
                        robot.outtake.shootCloseAction(robot)
                    ),
//
                    new ParallelAction(
                        robot.outtake.shootFarAction(robot),
//                        robot.transfer.intakeTransferTimeAction(SHOOTER_TIME),
                        BotActions.transferHold(robot, SHOOTER_TIME),
                        new SleepAction(SHOOTER_TIME)

                    )
//                    human1,
//                    human1_return,
//                    human2,
//                    human2_return
                )
            )
        );
    }

}