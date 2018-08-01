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
    
    private DiffSwerveModule mod;
    private Joystick velStick;
    private Joystick rotStick;
   
    public ModuleClosedLoopCommand(Joystick vel, Joystick spin) {
        mod = Robot.runningrobot.mod;
        velStick = vel;
        rotStick = spin;
    }
    
    @Override
    public void execute() {
        double vel;
        int target;
        if(MathUtil.outOfDeadband(velStick, DEADBAND)){
             vel = MathUtil.adjustDeadband(velStick, DEADBAND, true, false).getY();
        }
        else {
            vel = 0.0;
        }
        
        if(MathUtil.outOfDeadband(rotStick, DEADBAND)) {
             target = (int) (MathUtil.wrapAngleRad(rotStick.getDirectionRadians())/(2 * Math.PI) * DiffSwerveModule.STEERING_COUNTS_PER_REV);
        }
        else {
             target = mod.getModulePositionTrunc();
        }
        
        
        SmartDashboard.putNumber("Target", target);
        mod.setPositionAndSpeed(vel, target);
    }

    
    @Override
    protected boolean isFinished() {
        return false;
    }

}