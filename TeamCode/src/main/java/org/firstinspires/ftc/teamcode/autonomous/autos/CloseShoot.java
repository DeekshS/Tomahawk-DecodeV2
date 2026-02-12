package org.firstinspires.ftc.teamcode.autonomous.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.OuttakeConstants;
import org.firstinspires.ftc.teamcode.autonomous.autos.BotActions;
import org.firstinspires.ftc.teamcode.autonomous.autos.FCV2;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
@Config
public class CloseShoot extends LinearOpMode implements FCV2 {




    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
            Robot.outtake.shootCloseAction()
        );
//        robot.drive.localizer.update();
//        PoseStorage.endPose = robot.drive.localizer.getPose();
//        PoseStorage.side = PoseStorage.SIDE.BLUE;
    }

}