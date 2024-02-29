package frc.robot.Auton.Commands.Pizzabox;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePizzabox;

public class prepShootNote extends Commands {
    @Override
    public void initialize() {
        IntakePizzabox.getInstance().setState(IntakePizzaBox.PizzaboxState.SHOOT_PREP);
    }
}