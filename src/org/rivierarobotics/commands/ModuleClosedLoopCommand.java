package org.rivierarobotics.commands;

import org.rivierarobotics.mathutil.MathUtil;
import org.rivierarobotics.mathutil.Vector2d;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.DiffSwerveModule;
import org.rivierarobotics.subsystems.DiffSwerveModule.ModuleID;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;


/**
 * Command for testing individual modules in closed loop
 */
public class ModuleClosedLoopCommand extends Command{
    
    public static final Vector2d DEADBAND = new Vector2d(.1, .1);
    public static final double kHeading = .01;
    
    private DiffSwerveModule mod1, mod2;
    private Joystick stick;
   
    public ModuleClosedLoopCommand(Joystick js) {
        requires(Robot.runningrobot.dt);
        mod1 = Robot.runningrobot.dt.getModule(ModuleID.FL);
        mod2 = Robot.runningrobot.dt.getModule(ModuleID.BR);
        stick = js;
    }
    
    @Override
    public void execute() {
        Vector2d transVec;
        
        if(MathUtil.outOfDeadband(stick, DEADBAND)) {
            transVec = MathUtil.adjustDeadband(stick, DEADBAND, false, false);
        }
        else {
            transVec = new Vector2d(0.0,0.0);
        }
        
        
        mod1.setToVectorSmart(transVec.scale(.5));
        mod2.setToVectorSmart(transVec.scale(.5));

    }

    
    @Override
    protected boolean isFinished() {
        return false;
    }

}