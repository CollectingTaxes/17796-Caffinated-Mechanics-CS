package org.firstinspires.ftc.teamcode.util;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class BetterGamepad extends GamepadEx {
    private double triggerThreshold = 0.3;
    private double stickThreshold = 0.7;

    /**
     * The constructor, that contains the gamepad object from the
     * opmode.
     *
     * @param gamepad the gamepad object from the opmode
     */
    public BetterGamepad(Gamepad gamepad) {
        super(gamepad);
    }

    /**
     * Sets the Trigger Threshold of the gamepad, which is how far the stick needs to be pressed
     * down to register as a press.
     *
     * @param triggerThreshold the threshold (ranges from 0 - 1)
     */
    public void setTriggerThreshold(double triggerThreshold) {
        this.triggerThreshold = MathUtils.clamp(triggerThreshold, 0, 1);
    }

    /**
     * Sets the Stick Threshold of the gamepad, which is how close the stick needs to be to the
     * edge to register from the methods {@link #getLeftStickTouchingEdge()} and
     * {@link #getRightStickTouchingEdge()}
     *
     * @param stickThreshold the threshold (ranges from 0 - 1)
     */
    public void setStickThreshold(double stickThreshold) {
        this.stickThreshold = MathUtils.clamp(stickThreshold, 0, 1);
    }

    /**
     * Uses the trigger threshold which can be set using {@link #setTriggerThreshold(double)}
     * @param trigger the gamepad trigger to check if it is being pressed
     * @return true if trigger is past threshold; false if not
     */
    public boolean getTriggerPressed(GamepadKeys.Trigger trigger) {
        return getTrigger(trigger) > triggerThreshold;
    }

    /**
     * Uses the stick threshold which can be set using {@link #setStickThreshold(double)} and the
     * distance from the center of the joystick and the current stick position
     * @return true if left stick is past threshold; false if not
     */
    public boolean getLeftStickTouchingEdge() {
        return (Math.sqrt((Math.pow(getLeftX(), 2)) + (Math.pow(getLeftY(), 2)))) >  stickThreshold;
    }

    /**
     * Uses the stick threshold which can be set using {@link #setStickThreshold(double)} and the
     * distance from the center of the joystick and the current stick position
     * @return true if right stick is past threshold; false if not
     */
    public boolean getRightStickTouchingEdge() {
        return (Math.sqrt((Math.pow(getRightX(), 2)) + (Math.pow(getRightY(), 2)))) >  stickThreshold;
    }

    /**
     * @param button the button to check
     * @return true if button is pressed; false if not
     */
    public boolean get(GamepadKeys.Button button) {
        return getButton(button);
    }

    /**
     * Uses the trigger threshold which can be set using {@link #setTriggerThreshold(double)}
     * @param trigger the gamepad trigger to check
     * @return true if trigger is past threshold; false if not
     */
    public boolean get(GamepadKeys.Trigger trigger) {
        return getTriggerPressed(trigger);
    }

    /**
     * For some reason, the left stick is set to negative, but the right one's
     * y value is not set to negative. this is strange so I overrode it.
     * I don't really see how this was intentional so I don't think it was
     */
    @Override
    public double getRightY() {
        return -gamepad.right_stick_y;
    }

    // TODO check what direction this function goes around whether its clockwise or counterclockwise
    // TODO and where it is. Update the java doc comment to match this

    /**
     * Since the stick is a circle, this function allows finding the angle theta
     * between the terminal side (the top) and the stick's current position as a polar coordinate
     * @return the angle theta in degrees between the top of the stick and left stick position
     */
    public double getLeftStickToDegrees() {
        return Math.toDegrees(Math.atan2(getLeftX(), -getLeftY()));
    }

    /**
     * Since the stick is a circle, this function allows finding the angle theta
     * between the terminal side (the top) and the stick's current position as a polar coordinate
     * @return the angle theta in degrees between the top of the stick and right stick position
     */
    public double getRightStickToDegrees() {
        return Math.toDegrees(Math.atan2(getRightX(), -getRightY()));
    }
}



