package org.rivierarobotics.commands;

import org.rivierarobotics.mathutil.MathUtil;
import org.rivierarobotics.mathutil.Vector2d;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Command for joystick control of swerve drivetrain
 */
public class SwerveControlCommand extends Command{
    
    public static final Vector2d DEADBAND = new Vector2d(.1, .1);
    public static final double kHeading = .01;
    
    private DriveTrain dt;
    private Joystick transStick;
    private Joystick rotStick;
    private double setHeading = Double.NaN;
    
    public SwerveControlCommand(Joystick trans, Joystick spin) {
        dt = Robot.runningrobot.dt;
        transStick = trans;
        rotStick = spin;
        requires(dt);
    }
    
    @Override
    public void execute() {
        Vector2d transVec;
        double spinVal;
        if(MathUtil.outOfDeadband(transStick, DEADBAND)){
            //fetch field-centered translation vector
             transVec = MathUtil.adjustDeadband(transStick, DEADBAND, true, false).rotate(Math.PI/2);
        }
        else {
            transVec = new Vector2d(0.0,0.0);
        }
        
        if(MathUtil.outOfDeadband(rotStick, DEADBAND)) {
            //fetch scalar-valued spin parameter
             spinVal = MathUtil.adjustDeadband(rotStick, DEADBAND, true, true).getX();
             setHeading = dt.getGyroHeading();//in degrees for more intuitive P-gain
        }
        else {
            spinVal = 0.0;
            if(Double.isNaN(setHeading)) {
                spinVal = 0.0;
            }
            else {
                //if the driver is not trying to spin, try to keep the robot from twisting
                spinVal = kHeading*MathUtil.boundHalfAngleDeg(setHeading - dt.getGyroHeading());
            }
        }
        dt.swerve(spinVal, transVec);//lets swerve!
    }

    
    @Override
    protected boolean isFinished() {
        return false;
    }

}