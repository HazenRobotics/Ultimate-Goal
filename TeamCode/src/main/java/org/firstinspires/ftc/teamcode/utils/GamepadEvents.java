package org.firstinspires.ftc.teamcode.utils;

import android.view.KeyEvent;
import android.view.MotionEvent;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A gampad class that can access button events
 */
public class GamepadEvents {

    private Gamepad gamepad;

    /**
     * left analog stick horizontal axis
     */
    public float left_stick_x = 0f;

    /**
     * left analog stick vertical axis
     */
    public float left_stick_y = 0f;

    /**
     * right analog stick horizontal axis
     */
    public float right_stick_x = 0f;

    /**
     * right analog stick vertical axis
     */
    public float right_stick_y = 0f;

    /**
     * dpad up
     */
    public GamepadButton dpad_up = new GamepadButton();

    /**
     * dpad down
     */
    public GamepadButton dpad_down = new GamepadButton();

    /**
     * dpad left
     */
    public GamepadButton dpad_left = new GamepadButton();

    /**
     * dpad right
     */
    public GamepadButton dpad_right = new GamepadButton();

    /**
     * button a
     */
    public GamepadButton a = new GamepadButton();

    /**
     * button b
     */
    public GamepadButton b = new GamepadButton();

    /**
     * button x
     */
    public GamepadButton x = new GamepadButton();

    /**
     * button y
     */
    public GamepadButton y = new GamepadButton();

    /**
     * button guide - often the large button in the middle of the controller. The OS may
     * capture this button before it is sent to the app; in which case you'll never
     * receive it.
     */
    public GamepadButton guide = new GamepadButton();

    /**
     * button start
     */
    public GamepadButton start = new GamepadButton();

    /**
     * button back
     */
    public GamepadButton back = new GamepadButton();

    /**
     * button left bumper
     */
    public GamepadButton left_bumper = new GamepadButton();

    /**
     * button right bumper
     */
    public GamepadButton right_bumper = new GamepadButton();

    /**
     * left stick button
     */
    public GamepadButton left_stick_button = new GamepadButton();

    /**
     * right stick button
     */
    public GamepadButton right_stick_button = new GamepadButton();

    /**
     * left trigger
     */
    public float left_trigger = 0f;

    /**
     * right trigger
     */
    public float right_trigger = 0f;

    /**
     * PS4 Support - Circle
     */
    public GamepadButton circle = new GamepadButton();

    /**
     * PS4 Support - cross
     */
    public GamepadButton cross = new GamepadButton();

    /**
     * PS4 Support - triangle
     */
    public GamepadButton triangle = new GamepadButton();

    /**
     * PS4 Support - square
     */
    public GamepadButton square = new GamepadButton();

    /**
     * PS4 Support - share
     */
    public GamepadButton share = new GamepadButton();

    /**
     * PS4 Support - options
     */
    public GamepadButton options = new GamepadButton();

    /**
     * PS4 Support - touchpad
     */
    public GamepadButton touchpad = new GamepadButton();

    /**
     * PS4 Support - PS Button
     */
    public GamepadButton ps = new GamepadButton();



    public GamepadEvents ( Gamepad gamepad ) {
        this.gamepad = gamepad;
    }

    /**
     * Updates the button states from the gamepad
     */
    public void update() {
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
        dpad_down.update(gamepad.dpad_down);
        dpad_up.update(gamepad.dpad_up);
        dpad_right.update(gamepad.dpad_right);
        dpad_left.update(gamepad.dpad_left);

        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);
        guide.update(gamepad.guide);
        start.update(gamepad.start);

        back.update(gamepad.back);

        right_bumper.update(gamepad.right_bumper);
        left_bumper.update(gamepad.left_bumper);
        left_stick_button.update(gamepad.left_stick_button);
        right_stick_button.update(gamepad.right_stick_button);
    }



    public class GamepadButton {
        private boolean value;
        private boolean previous;
        private long pressedTime;
        private long heldTime;

        public GamepadButton() {
            previous = false;
            value = false;
        }


        /**
         * Checks if the button has been pressed
         * @return if the button has been pressed
         */
        public boolean onPress() {
            if(stateChanged() && value == true) {
                pressedTime = System.currentTimeMillis();
                return true;
            }
            return false;
        }

        /**
         * Checks if the button has been released
         * @return if the button has been released
         */
        public boolean onRelease() {
            return stateChanged() && value == false;
        }

        /**
         * Checks if the button has been held for an amount of time
         * @param millis amount of time
         * @return if the button has been held for the specified time
         */
        public boolean onHeldFor(long millis) {
            return heldTime >= millis;
        }

        /**
         * Gets the boolean value of the button
         * @return boolean value of the button
         */
        public boolean getValue() {
            return value;
        }

        /**
         * Checks if the button state has changed
         * @return if the button state has changed
         */
        private boolean stateChanged() {
            return value != previous;
        }

        /**
         * Updates the button state
         * @param value button boolean value
         */
        void update( boolean value) {
            this.previous = this.value;
            this.value = value;
            if(value == true) {
                heldTime = System.currentTimeMillis() - pressedTime;
            } else {
                heldTime = 0;
                pressedTime = 0;
            }
        }
    }


}
