package frc.robot.Auton.Commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class autoAlignArm extends Command {
    
    @Override
    public void initialize(){
        Arm.getInstance().armAutoAlign();
    }
    
    @Override
    public boolean isFinished() {
        boolean finished = Arm.getInstance().atTarget();
        return finished;
    }
}
