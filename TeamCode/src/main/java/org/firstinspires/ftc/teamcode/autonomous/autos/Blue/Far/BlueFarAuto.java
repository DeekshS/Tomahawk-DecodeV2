
 package org.firstinspires.ftc.teamcode.autonomous.autos.Blue.Far;


 import static org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants.FAR_HOOD1;
 import static org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants.FAR_VELOCITY1;

 import com.acmerobotics.dashboard.config.Config;
 import com.acmerobotics.roadrunner.Action;
 import com.acmerobotics.roadrunner.Arclength;
 import com.acmerobotics.roadrunner.ParallelAction;
 import com.acmerobotics.roadrunner.Pose2d;
 import com.acmerobotics.roadrunner.Pose2dDual;
 import com.acmerobotics.roadrunner.PosePath;
 import com.acmerobotics.roadrunner.SequentialAction;
 import com.acmerobotics.roadrunner.SleepAction;
 import com.acmerobotics.roadrunner.TranslationalVelConstraint;
 import com.acmerobotics.roadrunner.Vector2d;
 import com.acmerobotics.roadrunner.VelConstraint;
 import com.acmerobotics.roadrunner.ftc.Actions;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

 import org.firstinspires.ftc.teamcode.autonomous.autos.BotActions;
 import org.firstinspires.ftc.teamcode.autonomous.autos.FCV2;
 import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
 import org.firstinspires.ftc.teamcode.subsystems.Robot;
 import org.firstinspires.ftc.teamcode.PoseStorage;
 import org.jetbrains.annotations.NotNull;

 @Autonomous
 @Config
 public class BlueFarAuto extends LinearOpMode implements FCV2 {

     public static double INTAKE_WAIT_TIME = 1.4;

     public static int ARTIFACT_SHOOT_VEL = FAR_VELOCITY1;
     public static double HOOD_POS = FAR_HOOD1;
     public static double SHOOTER_TIME = 1.25;


     public void runOpMode() throws InterruptedException {
         Robot robot = new Robot(this);

         Robot.drive = new MecanumDrive(hardwareMap, FCV2.BLUE_FAR_START);

         MecanumDrive drive = Robot.drive;

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

         waitForStart();
         if (isStopRequested()) return;


         Actions.runBlocking(
             new ParallelAction(
                 Robot.outtake.hoodAction(HOOD_POS, 29.9),
//                 robot.outtake.shootVelocityTimeAction(ARTIFACT_SHOOT_VEL, 29.9),
                 Robot.intake.intakeTimeAction(29.9),
//                 robot.turret.alignAction(0, 29.9),
                 new SequentialAction(
                     new ParallelAction(
                         preload,
                         Robot.outtake.shootFarAction()
                     ),
//
                     new ParallelAction(
                         Robot.outtake.shootFarAction(),
                         BotActions.transferHold(SHOOTER_TIME),
                         new SleepAction(SHOOTER_TIME)

                     ),

                     Robot.transfer.transferStopAction(),

                     new ParallelAction(
                         gpp,
                         Robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                     ),
//
                     Robot.intake.stop(),


                     new ParallelAction(
                         gpp_return,
                         Robot.outtake.shootFarAction()
                     ),

                     new ParallelAction(
                         Robot.outtake.shootFarAction(),
                         BotActions.transferHold(SHOOTER_TIME),
                         new SleepAction(SHOOTER_TIME)

                     ),

                     Robot.transfer.transferStopAction(),

                     new ParallelAction(
                         human1,
                         Robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                     ),
//
                     Robot.intake.stop(),


                     new ParallelAction(
                         human1_return,
                         Robot.outtake.shootFarAction()
                     ),

                     new ParallelAction(
                         Robot.outtake.shootFarAction(),
                         BotActions.transferHold(SHOOTER_TIME),
                         new SleepAction(SHOOTER_TIME)

                     ),

                     Robot.transfer.transferStopAction(),

                     new ParallelAction(
                         human2,
                         Robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                     ),
//
                     Robot.intake.stop(),


                     new ParallelAction(
                         human2_return,
                         Robot.outtake.shootFarAction()
                     ),

                     new ParallelAction(
                         Robot.outtake.shootFarAction(),
                         BotActions.transferHold(SHOOTER_TIME),
                         new SleepAction(SHOOTER_TIME)

                     ),

                     Robot.transfer.transferStopAction(),

                     new ParallelAction(
                         human3,
                         Robot.intake.intakeTimeAction(INTAKE_WAIT_TIME)
                     ),
//
                     Robot.intake.stop(),


                     new ParallelAction(
                         human3_return,
                         Robot.outtake.shootFarAction()
                     ),

                     new ParallelAction(
                         Robot.outtake.shootFarAction(),
                         BotActions.transferHold(SHOOTER_TIME),
                         new SleepAction(SHOOTER_TIME)

                     ),

                     Robot.transfer.transferStopAction()
                 )
             )
         );
     }

 }