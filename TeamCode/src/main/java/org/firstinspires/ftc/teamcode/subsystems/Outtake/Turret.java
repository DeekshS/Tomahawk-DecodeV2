package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret {
    DcMotorEx turretMotor;

    public Turret(LinearOpMode mode) {
        turretMotor = mode.hardwareMap.get(DcMotorEx.class, "turret");
    }
}
