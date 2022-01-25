package org.firstinspires.ftc.teamcode.robot;

public class GamepadButton {

    private int toggleMax;
    public boolean isHeld;
    public boolean isJustPressed;
    public int toggle = 0;

    public GamepadButton(int toggleNum) { toggleMax = toggleNum; }

    public void update(boolean button) {
        isJustPressed = false;
        if (button && !isHeld) {
            toggle += 1;
            toggle %= toggleMax;
            isJustPressed = true;
        }
        isHeld = button;
    }
}
