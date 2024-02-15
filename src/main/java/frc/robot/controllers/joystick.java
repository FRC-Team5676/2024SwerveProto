package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class joystick {

    private final Joystick controller;
    public final JoystickButton button1;
    public final JoystickButton button2;
    private final double kDeadzoneStick = 0.1;

    /**
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public joystick(int port) {
        controller = new Joystick(port);
        button1 = createButton(1);
        button2 = createButton(2);
    }

    /** The X (left/right) position of the joystick on the controller from -1.0 to 1.0 
     * Strafe left/right */
    public double getStickX() {
        return deadzone(controller.getX(), kDeadzoneStick);
    }

    /** The Y (up/down) position of the joystick on the controller from -1.0 to 1.0
     * Move forward or backwards */
    public double getStickY() {
        return deadzone(controller.getY(), kDeadzoneStick);
    }

    /** The Z ()clockwise/counter-clockwise) rotation of the joystick on the controller from -1.0 to 1.0
     * Rotate robot clockwise or counter-clockwise*/
    public double getStickZ() {
        return deadzone(controller.getZ(), kDeadzoneStick * 2);
    }

    private JoystickButton createButton(int buttonID) {
        return new JoystickButton(this.controller, buttonID);
    }

    private double deadzone(double x, double dz) {
        if (Math.abs(x) > dz) return x;
        else return 0.0;
    }
}
