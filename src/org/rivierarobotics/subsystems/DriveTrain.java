package org.rivierarobotics.subsystems;

import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.robot.RobotConstants;
import org.rivierarobotics.subsystems.SwerveModule.ModuleID;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.rivierarobotics.commands.SwerveControlCommand;
import org.rivierarobotics.mathutil.MathUtil;
import org.rivierarobotics.mathutil.SwerveCalculator;
import org.rivierarobotics.mathutil.Vector2d;


public class DriveTrain extends Subsystem {

    private SwerveModule fl;
    //private SwerveModule fr;
    //private SwerveModule bl;
    private SwerveModule br;
    private PigeonIMU gyro;

    public DriveTrain() {
        fl = new SwerveModule(SwerveModule.ModuleID.FL);
        //fr = new SwerveModule(SwerveModule.ModuleID.FR);
        //bl = new SwerveModule(SwerveModule.ModuleID.BL);
        br = new SwerveModule(SwerveModule.ModuleID.BR);
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
        fl.setDrivePower(0.0);
        //fr.setDrivePower(0.0);
        //bl.setDrivePower(0.0);
        br.setDrivePower(0.0);
    }
    
    public SwerveModule getModule(ModuleID id) {
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
