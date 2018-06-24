package org.rivierarobotics.robot;

import org.rivierarobotics.drivers.Driver;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.SwerveModule.ModuleID;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser<String> chooser = new SendableChooser<>();
    
    public DriveTrain dt;
    public Driver driver;
    public static Robot runningrobot;

    public Robot() {
        runningrobot = this;
        dt = new DriveTrain();
        driver = new Driver();

    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        dt.resetGyro();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        autoSelected = chooser.getSelected();
        // autoSelected = SmartDashboard.getString("Auto Selector",
        // defaultAuto);
        System.out.println("Auto selected: " + autoSelected);
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        switch (autoSelected) {
            case customAuto:
                // Put custom auto code here
                break;
            case defaultAuto:
            default:
                // Put default auto code here
                break;
        }
    }
    
    @Override
    public void teleopInit() {
        dt.resetGyro();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        printSmartDash();
    }

    @Override
    public void disabledPeriodic() {
        printSmartDash();
    }
    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic() {
    }

    public void printSmartDash() {
        SmartDashboard.putNumber("FL pos", dt.getModule(ModuleID.FL).getPositionTrunc());
        SmartDashboard.putNumber("BR pos", dt.getModule(ModuleID.BR).getPositionTrunc());
        SmartDashboard.putNumber("FL rad", dt.getModule(ModuleID.FL).getPositionRad());
        SmartDashboard.putNumber("BR rad", dt.getModule(ModuleID.BR).getPositionRad());
        SmartDashboard.putNumber("FL raw", dt.getModule(ModuleID.FL).getPosition());
        SmartDashboard.putNumber("BR raw", dt.getModule(ModuleID.BR).getPosition());
        SmartDashboard.putNumber("Gyro", dt.getGyroHeading());
    }
}