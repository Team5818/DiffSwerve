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
    
    private static final int POSITION_PID_IDX = 0;
    private static final int POSITION_GAINS_SLOT = 0;
    public static final int STEERING_COUNTS_PER_REV = 800;
    private static final int REMOTE_0 = 0;
    private static final int REMOTE_1 = 1;
    private static final int TIMEOUT = 10;
    private static final double SMALL_NUMBER = .00001;
    public static final double kP_POS = 1.0/600.0*1023.0;
    public static final double kI_POS = 0.0;
    public static final double kD_POS = 0.0;

    private Vector2d positionVec;
    private WPI_TalonSRX motor1;
    private WPI_TalonSRX motor2;
    private ModuleID modID;

    public DiffSwerveModule(ModuleID id) {
        modID = id;
        switch (modID) {
            case FR:
                positionVec = new Vector2d(RobotConstants.ROBOT_WIDTH/2.0, RobotConstants.ROBOT_LENGTH/2.0);
                motor1 = new WPI_TalonSRX(RobotConstants.FR_MOTOR_1);
                motor2 = new WPI_TalonSRX(RobotConstants.FR_MOTOR_1);
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
        motor1.setSensorPhase(true);
        motor2.setSensorPhase(true);
        
        //set (M1_ENC + M2_ENC)/2 to be feedback sensor on M2
        motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,POSITION_PID_IDX, TIMEOUT);

        motor2.configRemoteFeedbackFilter(motor1.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, REMOTE_0, TIMEOUT);
        motor2.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.RemoteSensor0, TIMEOUT);
        motor2.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.QuadEncoder, TIMEOUT);
        motor2.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, POSITION_PID_IDX, TIMEOUT);
        
        //set status frames
        motor2.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, TIMEOUT);
        motor2.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, TIMEOUT);
        
        //set gains
        motor2.selectProfileSlot(POSITION_GAINS_SLOT, POSITION_PID_IDX);
        motor2.config_kP(POSITION_GAINS_SLOT, kP_POS, TIMEOUT);
        motor2.config_kI(POSITION_GAINS_SLOT, kI_POS,TIMEOUT);
        motor2.config_kD(POSITION_GAINS_SLOT, kD_POS,TIMEOUT);
        
        //invert aux pid output on follower
        motor2.configAuxPIDPolarity(true, TIMEOUT);

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
        return motor1.getSelectedSensorPosition(TIMEOUT);
    }
    
    public double getMotor2Pos() {
        return motor2.getSelectedSensorPosition(TIMEOUT);
    }
    
    public void zeroPosition() {
        motor1.setSelectedSensorPosition(0, 0, TIMEOUT);
        motor2.setSelectedSensorPosition(0, 0, TIMEOUT);
    }
    
    public int getModulePositionTrunc() {
        return MathUtil.boundHalfAngleNative(motor2.getSelectedSensorPosition(TIMEOUT), STEERING_COUNTS_PER_REV);
    }
    
    public void setPositionAndSpeed(double drive, int target) {
        int diff = MathUtil.boundHalfAngleNative(target - getModulePositionTrunc(), STEERING_COUNTS_PER_REV);
        double setpoint = getMotor2Pos() + diff;
        SmartDashboard.putNumber("setpoint", setpoint);
        motor2.set(ControlMode.Position, 200, DemandType.ArbitraryFeedForward, .5);
        motor1.follow(motor2, FollowerType.AuxOutput1);
        SmartDashboard.putNumber("error", motor2.getClosedLoopError(0));
    }

}
