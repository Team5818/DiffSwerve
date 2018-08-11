package org.rivierarobotics.subsystems;

import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.robot.RobotConstants;
import org.rivierarobotics.subsystems.DiffSwerveModule.ModuleID;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.rivierarobotics.commands.SwerveControlCommand;
import org.rivierarobotics.mathutil.MathUtil;
import org.rivierarobotics.mathutil.SwerveCalculator;
import org.rivierarobotics.mathutil.Vector2d;


public class DriveTrain extends Subsystem {

    private DiffSwerveModule fl;
    //private DiffSwerveModule fr;
    //private DiffSwerveModule bl;
    private DiffSwerveModule br;
    private PigeonIMU gyro;

    public DriveTrain() {
        fl = new DiffSwerveModule(DiffSwerveModule.ModuleID.FL);
        //fr = new DiffSwerveModule(DiffSwerveModule.ModuleID.FR);
        //bl = new DiffSwerveModule(DiffSwerveModule.ModuleID.BL);
        br = new DiffSwerveModule(DiffSwerveModule.ModuleID.BR);
        gyro = new PigeonIMU(RobotConstants.GYRO_PORT);
    }

    public void resetGyro() {
        gyro.setYaw(0.0, 10);
    }

    public double getGyroHeading() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return ypr[0];
    }

    public void swerve(double rot, Vector2d trans) {
        Vector2d[] swerveVecs = SwerveCalculator.calculateAllModules(Math.toRadians(getGyroHeading()), rot, trans, fl.getPosVec(), br.getPosVec());
        fl.setToVectorSmart(swerveVecs[0]);
        //fr.setToVectorSmart(swerveVecs[1]);
        //bl.setToVectorSmart(swerveVecs[2]);
        br.setToVectorSmart(swerveVecs[1]);//index of 3 with four mods
    }

    public void stop() {
        fl.stop();
        //fr.setDrivePower(0.0);
        //bl.setDrivePower(0.0);
        br.stop();
    }
    
    public void zeroAll() {
        fl.zeroModule();
        br.zeroModule();
    }
    
    public DiffSwerveModule getModule(ModuleID id) {
        switch (id) {
//           case FR:
//                return fr;
            case FL:
                return fl;
            case BR:
                return br;
//            case BL:
//                return bl;
             default:
                 return null;
        }
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new SwerveControlCommand(Robot.runningrobot.driver.leftJoy,
                Robot.runningrobot.driver.rightJoy));
    }

}