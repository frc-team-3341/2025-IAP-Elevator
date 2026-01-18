package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Servo;

public class javnishservo extends Command{
    Servo s;
    int time;

    public javnishservo(Servo s, int time) {
        this.s = s;
        this.time = time;

        addRequirements(s);
    }

    @Override
    public void initialize() {
        s.setPulseTimeMicroseconds(time);
        System.out.println("avnish is not a real person");
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
