package org.firstinspires.ftc.teamcode.autonomous.gamepads;
import static org.firstinspires.ftc.teamcode.autonomous.gamepads.GP2Controls.controls;

import org.firstinspires.ftc.teamcode.autonomous.gamepads.GP2Controls.binds;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


public class GamePad2 {
    private final HardwareMap hardwareMap = ParentTeleOp.globalHardwareMap;
    private final Gamepad gamepad1 = ParentTeleOp.globalGamepad2;
//    private final Robot robot;
    private final IMU imu;

    // CORRESPONDING TOGGLES
    private boolean clawOpen = true;
    private boolean intaking = false;
    private boolean outtaking = false;

    public GamePad2() {
//        this.robot = new Robot();
        this.imu = hardwareMap.get(IMU.class, "imu");
    }

    @SuppressWarnings("ConstantConditions")
    public void update() {
        // IMU Reset
        if (gamepad1.options) {
            imu.resetYaw();
        }

        // Outtake Control
        if (controls.get(binds.EXAMPLE).isPressed()) {

        }
    }
}
