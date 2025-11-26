package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IMUGyro {

    IMU imu;
//    PinpointIMU imu;

    public IMUGyro(OpMode mode) {

        imu = mode.hardwareMap.get(IMU.class, "imu");
//        imu = mode.hardwareMap.get(PinpointIMU.class, "pinpoint");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu.initialize(parameters);

    }

    public double getYaw(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit);
    }

    public void resetIMU() {imu.resetYaw();}





}
