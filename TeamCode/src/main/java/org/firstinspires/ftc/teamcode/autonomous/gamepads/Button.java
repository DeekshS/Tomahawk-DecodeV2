package org.firstinspires.ftc.teamcode.autonomous.gamepads;

import java.util.function.Supplier;

public class Button {
    private final Supplier<Boolean> buttonInput;
    private boolean previousState = false;
    private boolean currentState = false;

    public Button(Supplier<Boolean> buttonInput) {
        this.buttonInput = buttonInput;
    }

    public void update() {
        previousState = currentState;
        currentState = buttonInput.get();
    }

    public boolean isPressed() {
        return currentState;
    }

    public boolean wasJustPressed() {
        return !previousState && currentState;
    }

    public boolean wasJustReleased() {
        return previousState && !currentState;
    }
}