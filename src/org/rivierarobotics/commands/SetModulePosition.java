package org.rivierarobotics.commands;

import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.SwerveModule;
import org.rivierarobotics.subsystems.SwerveModule.ModuleID;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

public class SetModulePosition extends Command{
    private DriveTrain dt = Robot.runningrobot.dt;
    private int position;
    private SwerveModule mod;
    
    public SetModulePosition(int pos, ModuleID id) {
        position = pos;
        mod = dt.getModule(id);
        requires(dt);
    }
    
    @Override
    protected void initialize() {
        mod.setPosition(position);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
    
}
