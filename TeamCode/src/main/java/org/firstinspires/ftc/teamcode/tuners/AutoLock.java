package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.pid.MiniPID;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;

@TeleOp
@Config
public class AutoLock extends LinearOpMode {
    public static double pwr = 0.2;
    public static double p = 1;
    public static double i = 0;
    public static double d = 0;
    public static double targetAngle = 270;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Turret turret = new Turret(hardwareMap);
        CRServo left = hardwareMap.get(CRServo.class, "turretLeft");
        CRServo right = hardwareMap.get(CRServo.class, "turretRight");
        AnalogInput servoEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        MecanumDrive drive = new MecanumDrive(this.hardwareMap, new Pose2d(24, 24, 0));

        MiniPID pid = new MiniPID(p, i, d);
        pid.setOutputLimits(0.7);

        double initialAngle = 0;
        double angle = 0;
        double error;
        double power;
        boolean boundsHittingLeft;
        boolean boundsHittingRight;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.localizer.update();
            error = (angle > 180 && targetAngle > 180) ? (angle - (initialAngle + targetAngle)) % 360 : ((angle > 180 && targetAngle < 180) ? (360 - ((angle - (initialAngle + targetAngle))) % 360) : ((angle < 180 && targetAngle < 180) ? -((angle - (initialAngle + targetAngle)) % 360) : 360 + ((angle - (initialAngle + targetAngle)) % 360)));
//            targetAngle = turret.autoAlign(drive.localizer.getPose());
//            power = Math.min(Math.max(Math.log(error) / Math.log(360), 0), 1);
            power = 0.25 * Math.log1p(error);
            angle = (servoEncoder.getVoltage() / 3.3 * 360) % 360;

            boundsHittingLeft = angle > 180 && angle < 360 && targetAngle < 180;
            boundsHittingRight = angle < 180 && angle > 0  && targetAngle > 180;

            if (Math.abs(error) < 5 || Math.abs(Math.abs(error) - 360) < 5) {
                left.setPower(0);
                right.setPower(0);
            } else {
                if ((error > 180 || (error < 0 && error > -180) || (boundsHittingLeft)) && (!boundsHittingRight)) {
                    left.setPower(power);
                    right.setPower(power);
                } else {
                    left.setPower(-power);
                    right.setPower(-power);
                }
            }

            telemetry.addData("Angle", angle);
            telemetry.addData("Target Angle", targetAngle);
//            error = (error > 180 || ((error < 0 && error > -180) || (boundsHittingLeft)) && (!boundsHittingRight)) ? error : -error;
//            error = (error > 180 && error < 360) ? 360 - error - targetAngle: error;
            telemetry.addData("Error", error);
//            telemetry.addData("PID Output", pid.getOutput(Math.abs(error / 360), 0));
            telemetry.addData("pwr", power);
            telemetry.update();
        }
//        turret.leftServo.forceResetTotalRotation();
//        turret.rightServo.forceResetTotalRotation();
    }
}