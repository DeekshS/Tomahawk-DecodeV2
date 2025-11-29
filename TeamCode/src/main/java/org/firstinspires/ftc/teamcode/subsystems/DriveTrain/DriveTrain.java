package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.gamepad.GamepadMappings;
import org.firstinspires.ftc.teamcode.subsystems.IMUGyro;

public class DriveTrain {

    private final IMUGyro imu;

    private final DcMotorEx fl;
    private final DcMotorEx fr;
    private final DcMotorEx bl;
    private final DcMotorEx br;

    private final DcMotorEx[] motors;
    private final DcMotorEx[] reversedMotors;

    private final GamepadMappings controls;

    public State s = State.ROBOTCENTRIC;

    public DriveTrain(HardwareMap hardwareMap, GamepadMappings controls) {

        this.controls = controls;

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        // Correct IMU instantiation
        imu = new IMUGyro(hardwareMap);

        motors = new DcMotorEx[]{fl, fr, bl, br};
        reversedMotors = new DcMotorEx[]{bl, fl};

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        for (DcMotorEx motor : reversedMotors) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);
        }
    }

    public void update() {
        double x = controls.strafe;
        double y = -controls.drive;
        double rx = controls.turn;

        double flPower = y + x + rx;
        double frPower = y - x - rx;
        double blPower = y - x + rx;
        double brPower = y + x - rx;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }

    public void addTelemetry() {
        // Add any telemetry here if needed
    }

    public void resetIMU() {
        imu.resetIMU();
    }

    public enum State {
        ROBOTCENTRIC
    }
}
