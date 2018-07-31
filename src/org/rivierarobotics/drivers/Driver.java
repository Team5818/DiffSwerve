package org.rivierarobotics.drivers;

import org.rivierarobotics.robot.RobotConstants;

import edu.wpi.first.wpilibj.Joystick;

public class Driver {
    
    public Joystick leftJoy;
    public Joystick rightJoy;
    public Joystick buttons;
    
    public Driver() {
        leftJoy = new Joystick(RobotConstants.LEFT_JOYSTICK_PORT);
        rightJoy = new Joystick(RobotConstants.RIGHT_JOYSTICK_PORT);
        buttons = new Joystick(RobotConstants.BUTTONS_PORT);
    }
    
}
