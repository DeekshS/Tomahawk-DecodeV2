package org.firstinspires.ftc.teamcode.subsystems;

public interface Subsystem<State> {

    void setState(State state);

    State getState();


}
