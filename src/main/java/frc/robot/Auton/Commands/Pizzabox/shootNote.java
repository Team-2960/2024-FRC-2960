package frc.robot.Auton.Commands.Pizzabox;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePizzabox;

public class shootNote extends Commands {
    @Override
    public void initialize() {
        IntakePizzabox.getInstance().setState(IntakePizzaBox.PizzaboxState.SHOOT);
    }

    @Override
    public boolean isFinished() {
        return !IntakePizzabox.getInstance().isNotePresent;
    }

    @Override
    public void end(boolean interrupt) {
        IntakePizzabox.getInstance().setState(IntakePizzaBox.PizzaboxState.IDLE);
    }
}