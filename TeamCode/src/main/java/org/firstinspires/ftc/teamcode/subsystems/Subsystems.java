package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Subsystems {

    public DriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;
    public Turret turret;
    public LimelightCamera ll;

    public Subsystems(LinearOpMode mode) {

        driveTrain = new DriveTrain(mode);
        intake = new Intake(mode);

        turret = new Turret(mode);
        ll = new LimelightCamera(mode);
        outtake = new Outtake(mode, ll);

    }



}
