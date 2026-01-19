package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class DelayCommand extends Command {
    
    double seconds;
    double tracker;

    public DelayCommand(double seconds) {
        this.seconds = seconds;
    }
    
    @Override
    public void initialize() {
        tracker = 0;
    }

    @Override
    public void execute() {
        tracker++;
    }

    @Override
    public boolean isFinished() {
        return tracker >= 50*seconds;
    }   
}
