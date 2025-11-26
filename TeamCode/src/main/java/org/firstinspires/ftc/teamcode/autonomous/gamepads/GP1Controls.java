package org.firstinspires.ftc.teamcode.autonomous.gamepads;

import java.util.Map;

import com.qualcomm.robotcore.hardware.Gamepad;


public class GP1Controls {
    static Gamepad gamepad1 = ParentTeleOp.globalGamepad1;
    static Map<binds, Button> controls = Map.of(

//            binds.EXAMPLE, new Button(gamepad1.right_bumper)

    );

    enum binds {
        EXAMPLE
    }
}

