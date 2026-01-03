package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Servo extends SubsystemBase{
    
    PWM p;

    public Servo() {
        p = new PWM(Constants.OperatorConstants.PWM_CHANNEL);
    }

    public void setPulseTimeMicroseconds(int time) {
        p.setPulseTimeMicroseconds(time);
    }

    public void setPosition(int angle) {
        int time = (int) ((20.0/3)*(angle))+500;
        setPulseTimeMicroseconds(time);
    }

}
