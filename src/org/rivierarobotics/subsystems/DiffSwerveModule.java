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
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DiffSwerveModule {

    public enum ModuleID {
        FR, FL, BL, BR;
    }
    
    private static final int VELOCITY_PID_IDX = 0;
    private static final int POSITION_PID_IDX = 1;
    private static final int VELOCITY_GAINS_SLOT = 0;
    private static final int POSITION_GAINS_SLOT = 1;
    private static final double POSITION_FEEDBACK_SCALE = 0.5;
    private static final int REMOTE_0 = 0;
    private static final int REMOTE_1 = 1;
    private static final int TIMEOUT = 10;
    private static final double SMALL_NUMBER = .00001;
    public static final double kP_POS = 0.0;
    public static final double kI_POS = 0.0;
    public static final double kD_POS = 0.0;
    public static final double kP_VEL = 0.0;
    public static final double kI_VEL = 0.0;
    public static final double kD_VEL = 0.0;
    public static final double kF_VEL = 0.0;

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
        motor1.setInverted(true);
        motor2.setInverted(false);
        motor1.setSensorPhase(false);
        motor2.setSensorPhase(false);
        
        //set (M1_ENC + M2_ENC)/2 to be feedback sensor on M2
        motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        motor2.configRemoteFeedbackFilter(motor1.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, REMOTE_0, TIMEOUT);
        motor2.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, TIMEOUT);
        motor2.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, TIMEOUT);
        motor2.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, POSITION_PID_IDX, TIMEOUT);
        motor2.configSelectedFeedbackCoefficient(POSITION_FEEDBACK_SCALE, POSITION_PID_IDX, TIMEOUT);
        
        //set status frames
        motor2.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, TIMEOUT);
        motor2.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, TIMEOUT);
        
        //set gains
        motor2.selectProfileSlot(POSITION_GAINS_SLOT, POSITION_PID_IDX);
        motor2.config_kP(POSITION_GAINS_SLOT, kP_POS);
        motor2.config_kI(POSITION_GAINS_SLOT, kI_POS);
        motor2.config_kD(POSITION_GAINS_SLOT, kD_POS);
        
        //setup following on left so that aux PID ouput is inverted
        motor2.configAuxPIDPolarity(false);
        motor1.follow(motor2, FollowerType.AuxOutput1);

    }    

    public Vector2d getPosVec() {
        return positionVec;
    }

    public void setToVectorDumb(Vector2d drive) {
        double forward = drive.getMagnitude();
        double angle = drive.getAngle();
        motor2.set(ControlMode.PercentOutput, forward, DemandType.AuxPID, angle);
    }
}
