
package frc.robot.Auton.Commands.Pizzabox;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePizzaBox;

public class shootFastNote extends Command {
    @Override
    public void initialize() {
        IntakePizzaBox.getInstance().setState(IntakePizzaBox.PizzaboxState.FAST_SHOOT);
    }

    @Override
    public boolean isFinished() {
        return !IntakePizzaBox.getInstance().isNotePresent();
    }

    @Override
    public void end(boolean interrupt) {
        IntakePizzaBox.getInstance().setState(IntakePizzaBox.PizzaboxState.IDLE);
    }
}