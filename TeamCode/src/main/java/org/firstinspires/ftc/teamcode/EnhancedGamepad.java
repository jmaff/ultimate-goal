package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class EnhancedGamepad {
    public Gamepad delegate;

    public Button A = new Button();
    public Button Y = new Button();
    public Button X = new Button();
    public Button B = new Button();
    public Button LEFT_BUMPER = new Button();
    public Button RIGHT_BUMPER = new Button();
    public Button DPAD_LEFT = new Button();
    public Button DPAD_RIGHT = new Button();
    public Button DPAD_UP = new Button();
    public Button DPAD_DOWN = new Button();
    public Button START = new Button();
    public Button BACK = new Button();
    public Button LEFT_JOYSTICK_PUSH = new Button();
    public Button RIGHT_JOYSTICK_PUSH = new Button();

    public EnhancedGamepad(Gamepad delegate) {
        this.delegate = delegate;
    }

    public void update() {
        A.last = A.state;
        Y.last = Y.state;
        X.last = X.state;
        B.last = B.state;
        LEFT_BUMPER.last = LEFT_BUMPER.state;
        RIGHT_BUMPER.last = RIGHT_BUMPER.state;
        DPAD_LEFT.last = DPAD_LEFT.state;
        DPAD_RIGHT.last = DPAD_RIGHT.state;
        DPAD_UP.last = DPAD_UP.state;
        DPAD_DOWN.last = DPAD_DOWN.state;
        START.last = START.state;
        BACK.last = BACK.state;
        LEFT_JOYSTICK_PUSH.last = LEFT_JOYSTICK_PUSH.state;
        RIGHT_JOYSTICK_PUSH.last = RIGHT_JOYSTICK_PUSH.state;

        A.state = delegate.a;
        Y.state = delegate.y;
        X.state = delegate.x;
        B.state = delegate.b;
        LEFT_BUMPER.state = delegate.left_bumper;
        RIGHT_BUMPER.state = delegate.right_bumper;
        DPAD_LEFT.state = delegate.dpad_left;
        DPAD_RIGHT.state = delegate.dpad_right;
        DPAD_UP.state = delegate.dpad_up;
        DPAD_DOWN.state = delegate.dpad_down;
        START.state = delegate.start;
        BACK.state = delegate.back;
        LEFT_JOYSTICK_PUSH.state = delegate.left_stick_button;
        RIGHT_JOYSTICK_PUSH.state = delegate.right_stick_button;
    }

    public static class Button {
        public boolean state = false;
        public boolean last = false;
        public boolean pressed() {
            return state && !last;
        }
    }

}
