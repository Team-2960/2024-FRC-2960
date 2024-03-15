package frc.robot.Auton.Commands.Arm;


import edu.wpi.first.math.geometry.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class armToPreset extends Command {
    private String preset_name;

    public armToPreset(String preset_name) {
        this.preset_name = preset_name;
    }

    @Override
    public void initialize() {
        Arm.getInstance().setState(preset_name);
    }

    @Override
    public boolean isFinished() {
        boolean finished = Arm.getInstance().atTarget();

        if (finished) {
            System.out.println("arm Preset isFinished True");
        } else{ 
            
            System.out.println("arm Preset isFinished False");
        }

        return finished;
    }

    @Override
    public void end(boolean interrupt) {
        System.out.println("arm Preset End");
    }
}