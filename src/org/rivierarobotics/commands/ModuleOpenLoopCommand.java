package org.rivierarobotics.commands;

import org.rivierarobotics.mathutil.MathUtil;
import org.rivierarobotics.mathutil.Vector2d;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.DiffSwerveModule;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

public class ModuleOpenLoopCommand extends Command{
    
    public static final Vector2d DEADBAND = new Vector2d(.1, .1);
    public static final double kHeading = .01;
    
    private DiffSwerveModule mod;
    private Joystick velStick;
    private Joystick rotStick;
   
    public ModuleOpenLoopCommand(Joystick vel, Joystick spin) {
        mod = Robot.runningrobot.mod;
        velStick = vel;
        rotStick = spin;
    }
    
    @Override
    public void execute() {
        double vel;
        double rot;
        if(MathUtil.outOfDeadband(velStick, DEADBAND)){
             vel = .8;//MathUtil.adjustDeadband(velStick, DEADBAND, true, false).getY();
        }
        else {
            vel = 0.0;
        }
        
        if(MathUtil.outOfDeadband(rotStick, DEADBAND)) {
             rot = MathUtil.adjustDeadband(rotStick, DEADBAND, true, false).getX();
        }
        else {
             rot = 0.0;
        }
        double p1 = vel + rot;
        double p2 = vel - rot;
        mod.setMotor1Power(p1);
        mod.setMotor2Power(p2);
    }

    
    @Override
    protected boolean isFinished() {
        return false;
    }

}