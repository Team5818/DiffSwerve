package org.rivierarobotics.subsystems;

import org.rivierarobotics.mathutil.MathUtil;
import org.rivierarobotics.mathutil.Vector2d;
import org.rivierarobotics.robot.RobotConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DiffSwerveModule {

    public enum ModuleID {
        FR, FL, BL, BR;
    }
    
    private static final int POSITION_MOTION_MAGIC_IDX = 0;
    private static final int POSITION_GAINS_SLOT = 0;
    public static final int STEERING_COUNTS_PER_REV = 800;
    private static final int REMOTE_0 = 0;
    private static final int REMOTE_1 = 1;
    private static final int TIMEOUT = 10;
    private static final double SMALL_NUMBER = .00001;
    public static final double kF = .0006777 * 1023.0;
    public static final double kP_POS = 1.0/300.0*1023.0;
    public static final double kI_POS = 0.0;
    public static final double kD_POS = 0.0;
    public static final int CRUISE_VEL = 1200;
    public static final int ACCEL = 2400;

    private Vector2d positionVec;
    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;
    private ModuleID modID;
    private double positionOffset;

    public DiffSwerveModule(ModuleID id) {
        modID = id;
        switch (modID) {
            case FR:
                positionVec = new Vector2d(RobotConstants.ROBOT_WIDTH/2.0, RobotConstants.ROBOT_LENGTH/2.0);
                motor1 = new WPI_TalonSRX(RobotConstants.FR_MOTOR_1);
                motor2 = new WPI_TalonSRX(RobotConstants.FR_MOTOR_2);
                break;
            case FL:
                positionVec = new Vector2d(-RobotConstants.ROBOT_WIDTH/2.0, RobotConstants.ROBOT_LENGTH/2.0);
                motor1 = new WPI_TalonSRX(RobotConstants.FL_MOTOR_1);
                motor2 = new WPI_TalonSRX(RobotConstants.FL_MOTOR_2);
                break;
            case BR:
                positionVec = new Vector2d(RobotConstants.ROBOT_WIDTH/2.0, -RobotConstants.ROBOT_LENGTH/2.0);
                motor1 = new WPI_TalonSRX(RobotConstants.BR_MOTOR_1);
                motor2 = new WPI_TalonSRX(RobotConstants.BR_MOTOR_2);
                break;
            case BL:
            default:
                positionVec = new Vector2d(-RobotConstants.ROBOT_WIDTH/2.0, -RobotConstants.ROBOT_LENGTH/2.0);
                motor1 = new WPI_TalonSRX(RobotConstants.BL_MOTOR_1);
                motor2 = new WPI_TalonSRX(RobotConstants.BL_MOTOR_2);
                break;
        }
        
        //polarities
        motor1.setInverted(false);
        motor2.setInverted(false);
        motor1.setSensorPhase(false);
        motor2.setSensorPhase(false);
        
        //set (M1_ENC + M2_ENC)/2 to be feedback sensor on M2
        motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,POSITION_MOTION_MAGIC_IDX, TIMEOUT);

        motor2.configRemoteFeedbackFilter(motor1.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, REMOTE_0, TIMEOUT);
        motor2.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, TIMEOUT);
        motor2.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, TIMEOUT);
        motor2.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, POSITION_MOTION_MAGIC_IDX, TIMEOUT);
        
        //set status frames
        motor2.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, TIMEOUT);
        motor2.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, TIMEOUT);
        
        //set gains
        motor2.selectProfileSlot(POSITION_GAINS_SLOT, POSITION_MOTION_MAGIC_IDX);
        motor2.config_kF(POSITION_GAINS_SLOT, kF, TIMEOUT);
        motor2.config_kP(POSITION_GAINS_SLOT, kP_POS, TIMEOUT);
        motor2.config_kI(POSITION_GAINS_SLOT, kI_POS,TIMEOUT);
        motor2.config_kD(POSITION_GAINS_SLOT, kD_POS,TIMEOUT);
        motor2.configMotionAcceleration(ACCEL,TIMEOUT);
        motor2.configMotionCruiseVelocity(CRUISE_VEL,TIMEOUT);
        
        //invert aux pid output on follower
        motor2.configAuxPIDPolarity(false, TIMEOUT);
        motor1.configAuxPIDPolarity(false, TIMEOUT);
    }    

    public Vector2d getPosVec() {
        return positionVec;
    }
    
    public void setMotor1Power(double pow) {
        motor1.set(ControlMode.PercentOutput, pow);
    }
    
    public void setMotor2Power(double pow) {
        motor2.set(ControlMode.PercentOutput, pow);
    }

    public double getMotor1Pos() {
        return motor1.getSelectedSensorPosition(POSITION_MOTION_MAGIC_IDX);
    }
    
    public double getMotor2Pos() {
        return motor2.getSelectedSensorPosition(POSITION_MOTION_MAGIC_IDX);
    }
    
    public void zeroPosition() {
        positionOffset = motor2.getSelectedSensorPosition(POSITION_MOTION_MAGIC_IDX);
    }
    
    public double getModuleVel() {
        return motor2.getSelectedSensorVelocity(POSITION_MOTION_MAGIC_IDX);
    }
    
    public int getModulePositionTrunc() {
        return MathUtil.boundHalfAngleNative(motor2.getSelectedSensorPosition(POSITION_MOTION_MAGIC_IDX), STEERING_COUNTS_PER_REV);
    }
    
    public void stop() {
        motor1.set(ControlMode.PercentOutput, 0.0);
        motor2.set(ControlMode.PercentOutput, 0.0);
    }
    
    public void setPositionAndSpeed(double drive, int target) {
        int diff = MathUtil.boundHalfAngleNative(target - (int)getMotor2Pos(), STEERING_COUNTS_PER_REV);
        double setpoint = getMotor2Pos() + diff;
        SmartDashboard.putNumber("Differentce:", diff);
        SmartDashboard.putNumber("setpoint", setpoint);
        motor2.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, drive);
        motor1.set(ControlMode.Follower, motor2.getDeviceID(), DemandType.ArbitraryFeedForward, -2*drive);
        SmartDashboard.putNumber("M1 pow", motor1.getMotorOutputPercent());
        SmartDashboard.putNumber("M2 pow", motor2.getMotorOutputPercent());
        SmartDashboard.putNumber("error", motor2.getClosedLoopError(0));
    }

}
