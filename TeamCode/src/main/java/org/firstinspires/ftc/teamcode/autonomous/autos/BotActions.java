package org.firstinspires.ftc.teamcode.autonomous.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.subsystems.Subsystems;

public class BotActions {

    Subsystems subsystems;

    public static double INTAKE_WAIT_TIME = 3;
    public static double SHOOTER_TIME = 2.5;



    public BotActions(Subsystems subsystems) {
        this.subsystems = subsystems;
    }

    public Action preload_parallel_blue(Action path) {

        return new ParallelAction(
                path,
                subsystems.outtake.shoot_close()
//                subsystems.turret.blue_init()
        );
    }


    public Action preload_parallel_red(Action path) {

        return new ParallelAction(
                path,
                subsystems.outtake.shoot_close()
//                subsystems.turret.red_init()
        );
    }

    public Action shoot_parallel() {

        return new ParallelAction(
//                subsystems.outtake.shoot_close_time(SHOOTER_TIME),
                subsystems.intake.intake(SHOOTER_TIME)
        );
    }

    public Action shoot_close_spin_up() {

        return new SequentialAction(
                subsystems.outtake.shoot_close()
//                subsystems.intake.intake(SHOOTER_TIME)
        );
    }



    public Action intake_parallel(Action path) {

        return new ParallelAction(
                path,
                subsystems.intake.intake(INTAKE_WAIT_TIME),
                subsystems.outtake.shoot_reverse(INTAKE_WAIT_TIME)
        );

    }


    public Action return_parallel(Action path) {

        return new ParallelAction(
                path,
                subsystems.outtake.shoot_reverse(1)
        );

    }



}
