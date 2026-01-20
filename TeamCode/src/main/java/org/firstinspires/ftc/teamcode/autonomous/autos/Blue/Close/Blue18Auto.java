package org.firstinspires.ftc.teamcode.autonomous.autos.Blue.Close;


import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.autonomous.autos.FCV3;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;




@Autonomous
@Config
public class Blue18Auto extends LinearOpMode implements FCV3 {


    public static double INTAKE_TIME = 1.5;
    public static double SHOOT_TIME = 0.7;
    public static double GATE_X = 17.5;
    public static double GATE_Y = 2;
    public static int ARTIFACT_SHOOT_VEL = 1700;
    public static double HOOD_POS = 0.65;
    public static double ANGLE = -35;
    public static double REV_TRANSFER_POWER = 0;




    public void runOpMode() throws InterruptedException {


        ElapsedTime e = new ElapsedTime();
        Robot robot = new Robot(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, BLUE_CLOSE_START);
        Action preload = drive.actionBuilder(FCV3.BLUE_CLOSE_START)
                .strafeToLinearHeading(FCV3.BLUE_CLOSE_SHOOT, FCV3.BLUE_CLOSE_ANGLE)
                .build();


        Action artifact1 = drive.actionBuilder(new Pose2d(FCV3.BLUE_CLOSE_SHOOT.x, FCV3.BLUE_CLOSE_SHOOT.y, FCV3.BLUE_CLOSE_ANGLE))
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(FCV3.PGP_BLUE_ARTIFACT.x, FCV3.PGP_BLUE_ARTIFACT.y, FCV3.BLUE_CLOSE_ANGLE), Math.toRadians(90))
                .build();


        Action artifact1_return = drive.actionBuilder(new Pose2d(FCV3.PGP_BLUE_ARTIFACT, FCV3.BLUE_ARTIFACT_ANGLE))


                .setTangent(Math.toRadians(285))
                .splineToLinearHeading(new Pose2d(FCV3.BLUE_CLOSE_SHOOT.x, FCV3.BLUE_CLOSE_SHOOT.y, FCV3.BLUE_CLOSE_ANGLE), Math.toRadians(45))
                .build();




        Action artifact2 = drive.actionBuilder(new Pose2d(FCV3.BLUE_CLOSE_SHOOT.x, FCV3.BLUE_CLOSE_SHOOT.y, FCV3.BLUE_CLOSE_ANGLE))
                .strafeTo(FCV3.PPG_BLUE_ARTIFACT)
                .build();


        Action artifact2_return = drive.actionBuilder(new Pose2d(FCV3.PPG_BLUE_ARTIFACT.x, FCV3.PPG_BLUE_ARTIFACT.y, FCV3.BLUE_ARTIFACT_ANGLE))


                .strafeToLinearHeading(FCV3.BLUE_CLOSE_SHOOT, FCV3.BLUE_CLOSE_ANGLE)
                .build();


        Action gate_score = drive.actionBuilder(new Pose2d(FCV3.BLUE_CLOSE_SHOOT.x, FCV3.BLUE_CLOSE_SHOOT.y, FCV3.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(new Vector2d(FCV3.BLUE_GATE.x - 14, FCV3.BLUE_GATE.y), 0)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(FCV3.BLUE_GATE_INTAKE.x - 20, 62, 0), 0)
                .build();


        Action gate_return = drive.actionBuilder(new Pose2d(FCV3.BLUE_GATE_INTAKE.x - 20, 62, Math.toRadians(0)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(FCV3.BLUE_CLOSE_SHOOT.x, FCV3.BLUE_CLOSE_SHOOT.y, FCV3.BLUE_CLOSE_ANGLE), Math.toRadians(90))


                .build();


        Action gate_score2 = drive.actionBuilder(new Pose2d(FCV3.BLUE_CLOSE_SHOOT.x, FCV3.BLUE_CLOSE_SHOOT.y, FCV3.BLUE_CLOSE_ANGLE))
                .strafeToLinearHeading(new Vector2d(FCV3.BLUE_GATE.x - 14, FCV3.BLUE_GATE.y), 0)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(FCV3.BLUE_GATE_INTAKE.x - 20, 62, 0), 0)
                .build();


        Action gate_return2 = drive.actionBuilder(new Pose2d(FCV3.BLUE_GATE_INTAKE.x - 20, 62, Math.toRadians(0)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(FCV3.BLUE_CLOSE_SHOOT.x, FCV3.BLUE_CLOSE_SHOOT.y, FCV3.BLUE_CLOSE_ANGLE), Math.toRadians(90))


                .build();


//        Action gate_score2 = drive.actionBuilder(new Pose2d(FCV3.BLUE_CLOSE_SHOOT.x, FCV3.BLUE_CLOSE_SHOOT.y, FCV3.BLUE_CLOSE_ANGLE))
//            .setTangent(Math.toRadians(180))
//            .splineToLinearHeading(new Pose2d(FCV3.BLUE_GATE.x - GATE_X, FCV3.BLUE_GATE.y+GATE_Y, Math.toRadians(65)), Math.toRadians(90))
//            .build();
//
//        Action gate_return2 = drive.actionBuilder(new Pose2d(FCV3.BLUE_GATE.x - GATE_X, FCV3.BLUE_GATE.y+GATE_Y, Math.toRadians(65)))
//            .setTangent(Math.toRadians(180))
//            .splineToLinearHeading(new Pose2d(FCV3.BLUE_CLOSE_SHOOT.x, FCV3.BLUE_CLOSE_SHOOT.y, FCV3.BLUE_CLOSE_ANGLE), Math.toRadians(90))
//            .build();




        Action artifact3 = drive.actionBuilder(new Pose2d(FCV3.BLUE_CLOSE_SHOOT.x, FCV3.BLUE_CLOSE_SHOOT.y, FCV3.BLUE_CLOSE_ANGLE))
                .setTangent(Math.toRadians(190))
                .splineToSplineHeading(new Pose2d(FCV3.GPP_BLUE_ARTIFACT.x, FCV3.GPP_BLUE_ARTIFACT.y, FCV3.BLUE_CLOSE_ANGLE), Math.toRadians(90))
                .build();



        Action artifact3_return = drive.actionBuilder(new Pose2d(FCV3.GPP_BLUE_ARTIFACT.x, FCV3.GPP_BLUE_ARTIFACT.y, FCV3.BLUE_ARTIFACT_ANGLE))
                .strafeToLinearHeading(FCV3.BLUE_CLOSE_SHOOT, FCV3.BLUE_CLOSE_ANGLE)
                .build();


        Action p = new SequentialAction(
                robot.transfer.powerAction(-REV_TRANSFER_POWER),
                preload,
                robot.transfer.powerAction(1),
                new SleepAction(SHOOT_TIME)
        );


        Action a1 = new SequentialAction(
                robot.transfer.powerAction(-REV_TRANSFER_POWER),
                artifact1,
                artifact1_return,
                robot.transfer.powerAction(1),
                new SleepAction(SHOOT_TIME)
        );


        Action a2 = new SequentialAction(
                robot.transfer.powerAction(-REV_TRANSFER_POWER),
                artifact2,
                artifact2_return,
                robot.transfer.powerAction(1),
                new SleepAction(SHOOT_TIME)
        );


        Action gate = new SequentialAction(
                robot.transfer.powerAction(-REV_TRANSFER_POWER),
                gate_score,
                new SleepAction(INTAKE_TIME),
                gate_return,
                robot.transfer.powerAction(1),
                new SleepAction(SHOOT_TIME),
                robot.transfer.powerAction(-REV_TRANSFER_POWER),
                gate_score2,
                new SleepAction(INTAKE_TIME),
                gate_return2,
                robot.transfer.powerAction(1),
                new SleepAction(SHOOT_TIME)
        );


        Action a3 = new SequentialAction(
                robot.transfer.powerAction(-REV_TRANSFER_POWER),
                artifact3,
                artifact3_return,
                robot.transfer.powerAction(1),
                new SleepAction(SHOOT_TIME)
        );




        waitForStart();
        if (isStopRequested()) return;


        Actions.runBlocking(
                new ParallelAction(
                        robot.outtake.shootVelocityTimeAction(0, 29.9),
                        robot.intake.intakeTimeAction(29.9),
                        robot.turret.alignAction(ANGLE, 29.9),
                        robot.outtake.hoodAction(HOOD_POS, 29.9),
                        new SequentialAction(


                                // PRELOAD


                                p,


                                // ARTIFACT 1


                                a1,


                                // ARTIFACT 2


                                a2,


                                // GATE SCORES


                                gate,


                                // ARTIFACT 3


                                a3


                                // PARK
                        )
                )


        );
    }
}

