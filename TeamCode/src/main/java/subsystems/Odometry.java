package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import utility.RobotHardware;

public class Odometry extends SubsystemBase {
    // Hardware reference
    RobotHardware robot;

    public Odometry() {
        this.robot = RobotHardware.getInstance();
    }

    @Override
    public void periodic() {
    }
}
