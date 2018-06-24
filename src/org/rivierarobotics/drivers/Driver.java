package org.rivierarobotics.drivers;

import org.rivierarobotics.commands.SetModulePosition;
import org.rivierarobotics.commands.SetModulePower;
import org.rivierarobotics.robot.RobotConstants;
import org.rivierarobotics.subsystems.SwerveModule.ModuleID;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

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
