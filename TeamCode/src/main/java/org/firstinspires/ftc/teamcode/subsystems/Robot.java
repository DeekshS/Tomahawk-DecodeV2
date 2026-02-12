package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;

public class Robot {

    public static DriveTrain driveTrain;
    public static Intake intake;
    public static Intake transfer;
    public static Outtake outtake;
//    public static Turret turret;
    public static MecanumDrive drive;
    public Robot(LinearOpMode mode) {
        driveTrain = new DriveTrain(mode);
        intake = new Intake(mode);
        transfer = new Intake(mode);
//        turret = new Turret(mode);
        outtake = new Outtake(mode);
        transfer = new Intake(mode);
        drive = new MecanumDrive(mode.hardwareMap, PoseStorage.endPose);
    }

}