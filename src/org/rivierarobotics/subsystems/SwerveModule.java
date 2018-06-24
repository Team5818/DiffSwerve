package org.rivierarobotics.subsystems;

import org.rivierarobotics.mathutil.MathUtil;
import org.rivierarobotics.mathutil.Vector2d;
import org.rivierarobotics.robot.RobotConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveModule {

    public enum ModuleID {
        FR, FL, BL, BR;
    }

    private static final int CRUISE_VEL = 1800;
    private static final int ACCEL = 7200;
    private static final int MOTION_MAGIC_IDX = 0;
    private static final int SLOT_IDX = 0;
    private static final int TIMEOUT = 10;
    private static final double SMALL_NUMBER = .00001;
    public static final double kP = 0.0005*1023;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.000459*1023.0;
    public static final double cruise = 0.0;
    public static final double accel = 0.0;

    public static final int STEERING_COUNTS_PER_REV = 4096 * 2;
    public static final double STEERING_ENC_ZERO_FL = -3061 + 2048;
    public static final double STEERING_ENC_ZERO_FR = 0.0;
    public static final double STEERING_ENC_ZERO_BL = 0.0;
    public static final double STEERING_ENC_ZERO_BR = -3809 + 2048;

    public double steeringEncZero;
    private Vector2d positionVec;
    private WPI_TalonSRX wheel;
    private WPI_TalonSRX steering;
    private ModuleID modID;
    private final double zeroPos;

    public SwerveModule(ModuleID id) {
        modID = id;
        switch (modID) {
            case FR:
                positionVec = new Vector2d(RobotConstants.ROBOT_WIDTH/2.0, RobotConstants.ROBOT_LENGTH/2.0);
                wheel = new WPI_TalonSRX(RobotConstants.FR_DRIVE);
                steering = new WPI_TalonSRX(RobotConstants.FR_STEERING);
                zeroPos = STEERING_ENC_ZERO_FR;
                break;
            case FL:
                positionVec = new Vector2d(-RobotConstants.ROBOT_WIDTH/2.0, RobotConstants.ROBOT_LENGTH/2.0);
                wheel = new WPI_TalonSRX(RobotConstants.FL_DRIVE);
                steering = new WPI_TalonSRX(RobotConstants.FL_STEERING);
                zeroPos = STEERING_ENC_ZERO_FL;
                break;
            case BR:
                positionVec = new Vector2d(RobotConstants.ROBOT_WIDTH/2.0, -RobotConstants.ROBOT_LENGTH/2.0);
                wheel = new WPI_TalonSRX(RobotConstants.BR_DRIVE);
                steering = new WPI_TalonSRX(RobotConstants.BR_STEERING);
                zeroPos = STEERING_ENC_ZERO_BR;
                break;
            case BL:
            default:
                positionVec = new Vector2d(-RobotConstants.ROBOT_WIDTH/2.0, -RobotConstants.ROBOT_LENGTH/2.0);
                wheel = new WPI_TalonSRX(RobotConstants.BL_DRIVE);
                steering = new WPI_TalonSRX(RobotConstants.BL_STEERING);
                zeroPos = STEERING_ENC_ZERO_BL;
                break;
        }
        steering.setSensorPhase(true);
        if(modID == ModuleID.BR) {
            wheel.setInverted(true);
        }
        steering.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, MOTION_MAGIC_IDX, TIMEOUT);
        steering.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TIMEOUT);
        steering.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT);
        steering.selectProfileSlot(SLOT_IDX, MOTION_MAGIC_IDX);
        steering.config_kF(SLOT_IDX, kF, TIMEOUT);
        steering.config_kP(SLOT_IDX, kP, TIMEOUT);
        steering.config_kI(SLOT_IDX, kI, TIMEOUT);
        steering.config_kD(SLOT_IDX, kD, TIMEOUT);
        steering.configMotionCruiseVelocity((int)CRUISE_VEL, TIMEOUT);
        steering.configMotionAcceleration((int)ACCEL, TIMEOUT);
    }

    public Vector2d getPosVec() {
        return positionVec;
    }

    public int getPosition() {
        return steering.getSelectedSensorPosition(MOTION_MAGIC_IDX);
    }
    
    public int getVelocity() {
        return steering.getSelectedSensorVelocity(MOTION_MAGIC_IDX);
    }
    
    public int getPositionTrunc() {
        return MathUtil.boundHalfAngleNative(getPosition(), STEERING_COUNTS_PER_REV);
    }

    public double getPositionRad() {
        return MathUtil.boundHalfAngleRad((double)(getPosition() - zeroPos)/(double)STEERING_COUNTS_PER_REV * Math.PI * 2.0);
    }
    
    public void setPosition(int target) {
        int diff = MathUtil.boundHalfAngleNative(target - getPositionTrunc(), STEERING_COUNTS_PER_REV);
        steering.set(ControlMode.MotionMagic, getPosition() + diff);
    }

    public void setPositionRads(double ang) {
        double raw = MathUtil.wrapAngleRad(ang) / (2 * Math.PI) * STEERING_COUNTS_PER_REV + zeroPos;
        setPosition((int)raw);
        if(modID == ModuleID.FL) {
            SmartDashboard.putNumber("FL setpoint", (int)raw);
        }
        else {
            SmartDashboard.putNumber("BR setpoint", (int)raw);
        }
    }

    public void setDrivePower(double pow) {
        wheel.set(ControlMode.PercentOutput,pow);
    }
    
    public void setSteeringPower(double pow) {
        steering.set(ControlMode.PercentOutput, pow);
    }

    public void setToVectorDumb(Vector2d drive) {
        if(drive.getMagnitude() < SMALL_NUMBER) {
            setDrivePower(0.0);
            return;
        }
        setPositionRads(drive.getAngle());
        setDrivePower(drive.getMagnitude());
    }

    public void setToVectorSmart(Vector2d drive) {
        double pow = drive.getMagnitude();
        if(pow < SMALL_NUMBER) {
            setDrivePower(0.0);
            return;
        }
        if (Math.abs(MathUtil.boundHalfAngleRad(drive.getAngle() - getPositionRad())) > Math.PI/2.0) {
            drive = drive.scale(-1);
            pow *= -1;
        }
        setPositionRads(drive.getAngle());
        setDrivePower(pow);
    }

}
