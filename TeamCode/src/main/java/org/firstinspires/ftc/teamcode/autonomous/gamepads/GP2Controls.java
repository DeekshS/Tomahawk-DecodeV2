package org.firstinspires.ftc.teamcode.autonomous.gamepads;

import java.util.Map;
import com.qualcomm.robotcore.hardware.Gamepad;


public class GP2Controls {
    static Gamepad gamepad2 = ParentTeleOp.globalGamepad2;
    static Map<binds, Button> controls = Map.of(

            binds.EXAMPLE, new Button(() -> gamepad2.right_bumper)

    );
    public enum binds {
        EXAMPLE
    }
}

