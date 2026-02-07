package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadMappings {

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    //Write mappings here
    //---------Gamepad 1---------
    //Intake & Transfer- Right Trigger
    //Intake & Transfer Reverse - Left Trigger
    //---------Gamepad 2---------
    //FlywheelClose - A
    //FlywheelFar - Y
    //FlywheelOff - B
    //Auto Aim Toggle - Dpad Up (Not in Use)



    //=============== DRIVETRAIN ===============
    public static double drive = 0.0;
    public static double strafe = 0.0;
    public static double turn = 0.0;
    public Toggle reset;

    //=============== INTAKE ===============
    public Toggle intake;
    public Toggle intake2;

    public Toggle intakeReverse;
//    public Toggle servoBlocker;
    public Toggle transfer;
    public Toggle transferReverse;


    //=============== OUTTAKE ===============
    public Toggle flywheelClose1;
    public Toggle flywheelClose2;
    public Toggle flywheelClose3;
    public Toggle flywheelFar1;
    public Toggle flywheelFar2;

    public Toggle flywheelOff;
    public Toggle autoAim;
    public Toggle autoVelo;
    public Toggle turretAuto;
    public Toggle turretLeft;
    public Toggle turretRight;

    public GamepadMappings(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        //=========DRIVETRAIN======
        reset = new Toggle(false);

        //=============== INTAKE ===============
        intake = new Toggle(false);
        intake2 = new Toggle(false);

        intakeReverse = new Toggle(false);
//        servoBlocker = new Toggle(false);
        transfer = new Toggle(false);
        transferReverse = new Toggle(false);

        //=============== OUTTAKE ===============
        flywheelClose1 = new Toggle(false);
        flywheelClose2 = new Toggle(false);
        flywheelClose3 = new Toggle(false);

        flywheelFar1 = new Toggle(false);
        flywheelFar2 = new Toggle(false);

        flywheelOff = new Toggle(false);
        autoAim = new Toggle(false);
        autoVelo = new Toggle(false);

        turretAuto = new Toggle(false);

        turretLeft = new Toggle(false);
        turretRight = new Toggle(false);
    }

    public void joystickUpdate() {
        drive = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
    }

    public void intakeUpdate() {
        intake.update(gamepad1.right_trigger > 0.5);
        intake2.update(gamepad2.right_trigger > 0.5);

    }
    public void intakeReverseUpdate() {
        intakeReverse.update(gamepad1.left_trigger > 0.5);
    }
    public void outtakeUpdate() {
//        flywheelClose1.update(gamepad2.x);
        flywheelClose2.update(gamepad2.y);
//        flywheelClose3.update(gamepad2.b);

        flywheelFar1.update(gamepad2.dpad_left);
        flywheelFar2.update(gamepad2.dpad_right);

        flywheelOff.update(gamepad2.b);
        turretAuto.update(gamepad2.ps);
        turretLeft.update(gamepad2.left_bumper);
        turretLeft.update(gamepad2.right_bumper);

    }

    // v1 robot
    public void update() {
        joystickUpdate();

        intakeUpdate();
        reset.update(gamepad2.dpad_up);

//        servoBlocker.update(gamepad1.left_bumper);
        transferReverse.update(gamepad1.left_bumper);
        transfer.update(gamepad1.right_bumper);


        intakeReverseUpdate();
        outtakeUpdate();
    }

    public void resetControls(Toggle... toggles) {
        for (Toggle toggle : toggles) {
            toggle.set(false);
        }
    }
}

