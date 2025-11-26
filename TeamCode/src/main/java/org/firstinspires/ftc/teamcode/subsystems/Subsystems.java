package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;


public class Subsystems {

    public DriveTrain driveTrain;
    public Intake intake;
    public Turret outtake;
    public Outtake outtake;
    public LimelightCamera ll;

    public Subsystems(LinearOpMode mode) {

        driveTrain = new DriveTrain(mode);
        intake = new Intake(mode);

        turret = new org.firstinspires.ftc.teamcode.subsystems.Turret.Turret(mode);
        ll = new LimelightCamera(mode);
        outtake = new Turret(mode, ll);

    }



}
