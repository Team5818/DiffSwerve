package org.rivierarobotics.robot;

import org.rivierarobotics.commands.ModuleClosedLoopCommand;
import org.rivierarobotics.commands.ModuleOpenLoopCommand;
import org.rivierarobotics.drivers.Driver;
import org.rivierarobotics.subsystems.DiffSwerveModule;
import org.rivierarobotics.subsystems.DiffSwerveModule.ModuleID;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    
    public Driver driver;
    public DiffSwerveModule mod1;
    public DiffSwerveModule mod2;
    public ModuleClosedLoopCommand control;
    public static Robot runningrobot;

    public Robot() {
        runningrobot = this;
        mod1 = new DiffSwerveModule(DiffSwerveModule.ModuleID.FL);
        mod2 = new DiffSwerveModule(DiffSwerveModule.ModuleID.BR);
        driver = new Driver();
        control = new ModuleClosedLoopCommand(driver.leftJoy,driver.rightJoy);
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
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
        mod1.zeroModule();
        mod2.zeroModule();
        control.start();
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
        double p1 = mod1.getMotor1Pos();
        double p2 = mod1.getMotor2Pos();
        SmartDashboard.putNumber("Motor 1 Position", p1);
        SmartDashboard.putNumber("Motor 2 Position", p2);
        SmartDashboard.putNumber("Wrapped", mod1.getModulePositionTrunc());
        SmartDashboard.putNumber("Module vel", mod1.getModuleVel());
    }
}