package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;

@TeleOp
public class AutoLockV2 extends LinearOpMode {
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Turret turret = new Turret(hardwareMap);
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, new Pose2d(24, 24, 0));


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.localizer.update();
            turret.autoAlign(drive.localizer.getPose());
            turret.update();

            telemetry.addData("Angle", turret.getAngle());
            telemetry.addData("Target Angle", turret.getTargetAngle());
            telemetry.addData("Error", turret.getError());
            telemetry.addData("Power", turret.getPower());
            telemetry.update();
        }
    }
}