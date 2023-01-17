package frc.robot.commands;

import frc.robot.ButtonMonitor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
// import frc.robot.subsystems.UltraSonicSensor;

//Jordan 1/13/2023 change Command to CommandBase and resolve importation issues
public class UltraDrive extends CommandBase {

    private double m_driveDirection; // 1 = forward, -1 = backward
    private final double m_power = 0.5;
    private final int timeOut = 3;
    private ButtonMonitor buttonMonitor;
    private double realVelocity;

    public UltraDrive(Trigger cmdButton, double velocity) {
        buttonMonitor = new ButtonMonitor(cmdButton);
        realVelocity = velocity;
        
    }

    // public void timedDriving() {
    //     execute();
    // }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Realvelocity", realVelocity);
        if (buttonMonitor.checkButtonState() == ButtonMonitor.ButtonState.Active){
            Robot.driveBase.drive(realVelocity, realVelocity);
            // Robot.led.shot();
        }

    }

    @Override
    public boolean isFinished() {
        return (buttonMonitor.checkButtonState() == ButtonMonitor.ButtonState.Inactive);
    }

    @Override
    public void end(boolean interrupted) {
        this.end();
    }

    // @Override
    public void end() {
        Robot.driveBase.drive.tankDrive(0, 0);
    }
}