package org.firstinspires.ftc.teamcode.robot;

public class GamepadButton {

    private int toggleMax;
    public boolean isHeld;
    public int toggle = 0;

    public GamepadButton(int toggleNum) {toggleMax = toggleNum;}

    public void update(boolean button) {
        if (button && !isHeld) {
            toggle += 1;
            toggle %= toggleMax;
        }
        isHeld = button;
    }
}
