package org.rivierarobotics.commands;

import org.rivierarobotics.mathutil.MathUtil;
import org.rivierarobotics.mathutil.Vector2d;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.DiffSwerveModule;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ModuleClosedLoopCommand extends Command{
    
    public static final Vector2d DEADBAND = new Vector2d(.1, .1);
    public static final double kHeading = .01;
    
    private DiffSwerveModule mod1, mod2;
    private Joystick velStick;
    private Joystick rotStick;
   
    public ModuleClosedLoopCommand(Joystick vel, Joystick spin) {
        mod1 = Robot.runningrobot.mod1;
        mod2 = Robot.runningrobot.mod2;
        velStick = vel;
        rotStick = spin;
    }
    
    @Override
    public void execute() {
        double vel;
        Vector2d transVec;
        if(MathUtil.outOfDeadband(velStick, DEADBAND)){
             vel = MathUtil.adjustDeadband(velStick, DEADBAND, true, false).getY();
        }
        else {
            vel = 0.0;
        }
        
        if(MathUtil.outOfDeadband(rotStick, DEADBAND)) {
            transVec = MathUtil.adjustDeadband(rotStick, DEADBAND, false, false);
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