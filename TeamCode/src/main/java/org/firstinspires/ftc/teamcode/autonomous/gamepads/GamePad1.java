package org.firstinspires.ftc.teamcode.autonomous.gamepads;

import static org.firstinspires.ftc.teamcode.autonomous.gamepads.GP1Controls.controls;
import org.firstinspires.ftc.teamcode.autonomous.gamepads.GP1Controls.binds;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


public class GamePad1 {
    private final HardwareMap hardwareMap = ParentTeleOp.globalHardwareMap;
    private final Gamepad gamepad1 = ParentTeleOp.globalGamepad1;
    private final IMU imu;

    public GamePad1() {
        this.imu = hardwareMap.get(IMU.class, "imu");
    }

    public void update() {
        // IMU Reset
        if (gamepad1.options) {
            imu.resetYaw();
        }

        // Intake Control
        if (controls.get(binds.EXAMPLE).isPressed()) {
        }
    }

}
