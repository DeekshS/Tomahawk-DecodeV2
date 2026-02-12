////package org.firstinspires.ftc.teamcode.tuners;
////
////
////import com.acmerobotics.dashboard.FtcDashboard;
////import com.acmerobotics.dashboard.config.Config;
////import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
////import com.acmerobotics.roadrunner.Pose2d;
////import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import com.qualcomm.robotcore.hardware.AnalogInput;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.DcMotorEx;
////import com.qualcomm.robotcore.hardware.PIDFCoefficients;
////import com.qualcomm.robotcore.hardware.Servo;
////
////
////import org.firstinspires.ftc.teamcode.PoseStorage;
////import org.firstinspires.ftc.teamcode.R;
////import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
////import org.firstinspires.ftc.teamcode.pid.MiniPID;
////import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
////import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
////import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
////import org.firstinspires.ftc.teamcode.subsystems.Robot;
////
////
////@TeleOp
////@Config
////public class TurretTuner extends LinearOpMode {
////    public static double targetAngle = 0;
////    public static boolean s1enabled = false;
////    public static boolean s2enabled = false;
////    public static boolean negated = false;
////
////    public void runOpMode() {
//////        Servo s1 = hardwareMap.get(Servo.class, "turretLeft");
//////        Servo s2 = hardwareMap.get(Servo.class, "turretRight");
////        Turret turret = new Turret(this);
////        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "turretEncoderRight");
////        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////        Robot robot = new Robot(this);
////
////        waitForStart();
////        if (isStopRequested()) return;
////
////        while (opModeIsActive()) {
////            robot.drive.localizer.update();
////
////            turret.autoAlign(robot.drive.localizer.getPose());
////            turret.update();
////
////            telemetry.addData("Turret Pos", turret.getTargetAngle());
////            telemetry.addData("Calculated Angle", turret.getTargetAngle());
////            telemetry.addData("Current Position", turret.getPosition());
////
////            telemetry.update();
////        }
////
////
////    }
////}
////
//package org.firstinspires.ftc.teamcode.tuners;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//import org.firstinspires.ftc.teamcode.PoseStorage;
//import org.firstinspires.ftc.teamcode.R;
//import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
//import org.firstinspires.ftc.teamcode.pid.MiniPID;
//import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
//import org.firstinspires.ftc.teamcode.subsystems.Outtake.Turret;
//import org.firstinspires.ftc.teamcode.subsystems.Robot;
//
//
//@TeleOp
//@Config
//public class TurretTuner extends LinearOpMode {
//    public static double targetAngle = 0;
//    public static boolean s1enabled = false;
//    public static boolean s2enabled = false;
//    public static boolean negated = false;
//
//    public void runOpMode() {
////        Servo s1 = hardwareMap.get(Servo.class, "turretLeft");
////        Servo s2 = hardwareMap.get(Servo.class, "turretRight");
//        Turret turret = new Turret(this);
//        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "turretEncoderRight");
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        Robot robot = new Robot(this);
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            turret.setTargetAngle(targetAngle);
//            turret.update();
////
////            s1.setPosition((targetAngle + 181) * 0.00319444);
////            s2.setPosition((targetAngle + 181) * 0.00319444);
//
//            double robotX = robot.drive.localizer.getPose().position.x;
//            double robotY = robot.drive.localizer.getPose().position.y;
//
//            double deltaX = PoseStorage.goalX - robotX;
//            double deltaY = PoseStorage.goalY - robotY;
//            double calculatedAngle = Math.toDegrees(Math.atan2(deltaY, deltaX)) - Math.toDegrees(robot.drive.localizer.getPose().heading.toDouble());
//
//            telemetry.addData("Turret Pos", turret.getPosition());
//            telemetry.addData("Limelight Error", turret.limelight.getLatestResult().getTx());
//            telemetry.addData("Limelight Error", turret.getPosition() - turret.limelight.getLatestResult().getTx());
//            telemetry.addData("Calculated Angle", calculatedAngle);
//            telemetry.update();
//        }
//
//
//    }
//}
//
//
//
