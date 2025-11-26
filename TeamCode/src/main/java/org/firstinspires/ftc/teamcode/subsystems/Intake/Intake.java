package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    Telemetry telemetry;
    public DcMotorEx intakeMotor;
    public Servo blocker;

    public Intake(LinearOpMode mode) {

        intakeMotor = mode.hardwareMap.get(DcMotorEx.class, "intake");
        //intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intakeTime(double time) {
        ElapsedTime timer = new ElapsedTime();

        if (timer.seconds() < time) {
            intakeMotor.setPower(1);
        }
        else if (timer.seconds() >= time) {
            intakeMotor.setPower(0);
            timer.reset();
        }


    }

    public void intakeReverse(double time) {
        ElapsedTime timer = new ElapsedTime();

        if (timer.seconds() < time) {
            intakeMotor.setPower(-0.5);
        }
        else if (timer.seconds() >= time) {
            intakeMotor.setPower(0);
            timer.reset();
        }
    }






}
