package frc.robot.Auton.Commands.Arm;


import edu.wpi.first.math.geometry.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class armToPreset extends Commands {
    private String preset_name;

    public armToPreset(String preset_name) {
        this.preset_name = preset_name;
    }

    @Override
    public void initialize() {
        Arm.getInstance().setStage(preset_name);
    }

    @Override
    public boolean isFinished() {
        return Arm.getInstance().atTarget();
    }
}