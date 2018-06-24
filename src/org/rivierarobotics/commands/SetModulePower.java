package org.rivierarobotics.commands;

import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.SwerveModule;
import org.rivierarobotics.subsystems.SwerveModule.ModuleID;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

public class SetModulePower extends Command{
    private DriveTrain dt = Robot.runningrobot.dt;
    private double power;
    private SwerveModule mod;
    
    public SetModulePower(double pow, ModuleID id) {
        power = pow;
        mod = dt.getModule(id);
        requires(dt);
    }
    
    @Override
    protected void initialize() {
        mod.setSteeringPower(power);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
    
}
