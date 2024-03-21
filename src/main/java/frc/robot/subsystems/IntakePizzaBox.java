package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePizzaBox extends SubsystemBase {
    public enum PizzaboxState {
        IDLE,
        INTAKE,
        SHOOT,
        FAST_SHOOT,
        SHOOT_PREP,
        REVERSE
    }

    private static IntakePizzaBox intake = null;

    private TalonFX intakeRollers;

    private CANSparkFlex shooterTop;
    private CANSparkFlex shooterBot;

    private RelativeEncoder shootEncoder1;
    private RelativeEncoder shootEncoder2;

    private DigitalInput photoeye;

    private PizzaboxState state;

    private GenericEntry sb_state;
    private GenericEntry sb_shooterTopVolt;
    private GenericEntry sb_shooterBotVolt;
    private GenericEntry sb_shooterTopRate;
    private GenericEntry sb_shooterBotRate;
    private GenericEntry sb_intakeRollerVolt;
    private GenericEntry sb_intakeRollerCurrent;
    private GenericEntry sb_intakeRollerRate;
    private GenericEntry sb_notePresent;

    /**
     * Constructor
     */
    private IntakePizzaBox() {
        // Initialize Intake Motor
        intakeRollers = new TalonFX(Constants.intakeRollers);
        intakeRollers.setInverted(true);

        // Initialize Shooter Motors
        shooterTop = new CANSparkFlex(Constants.shooterTop, MotorType.kBrushless);
        shooterBot = new CANSparkFlex(Constants.shooterBot, MotorType.kBrushless);
        shooterTop.setInverted(true);
        shooterBot.follow(shooterTop, true);

        // Initialize Shooter Encoders
        shootEncoder1 = shooterTop.getEncoder();
        shootEncoder2 = shooterBot.getEncoder();

        // Initialize photoeye
        photoeye = new DigitalInput(3);

        // Initialize state
        state = PizzaboxState.IDLE;

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("Pizzabox", BuiltInLayouts.kList)
                .withSize(2, 6);

        sb_state = layout.add("State", state.name()).getEntry();
        sb_shooterTopVolt = layout.add("Shooter 1 Voltage", 0).getEntry();
        sb_shooterBotVolt = layout.add("Shooter 2 Voltage", 0).getEntry();
        sb_shooterTopRate = layout.add("Shooter 1 Rate", 0).getEntry();
        sb_shooterBotRate = layout.add("Shooter 2 Rate", 0).getEntry();
        sb_intakeRollerVolt = layout.add("Intake Roller Voltage", 0).getEntry();
        sb_intakeRollerRate = layout.add("Intake Roller Rate", 0).getEntry();
        sb_notePresent = layout.add("Note Present", false).getEntry();
        sb_intakeRollerCurrent = layout.add("Intake Roller Current", 0).getEntry();

    }

    /**
     * Sets the Pizzabox State
     * 
     * @param new pizzabox state
     */
    public void setState(PizzaboxState state) {
        this.state = state;
    }

    /**
     * Gets the pizzabox state
     * 
     * @return current pizza box state
     */
    public PizzaboxState getState() {
        return state;
    }

    /**
     * Checks if a game piece is present
     * 
     * @return true if a gamepiece is present, false otherwise
     */
    public boolean isNotePresent() {
        return photoeye.get();
    }

    /**
     * Subsystem periodic function
     */
    @Override
    public void periodic() {
        if (state == PizzaboxState.INTAKE) {
            // Check if a gamepiece is present
            double getIntakeCurrent = intakeRollers.getTorqueCurrent().getValueAsDouble();
            intakeRollers.setVoltage(Constants.intakeInVoltage);

            if (isNotePresent()) {
                intakeRollers.setVoltage(0);
            }else if (Math.abs(getIntakeCurrent) > Constants.intakeSlowCurrent) {
                intakeRollers.setVoltage(Constants.intakeSlowVoltage);
            }else{
                intakeRollers.setVoltage(Constants.intakeInVoltage);
            }
            

            if (isNotePresent()) {
                intakeRollers.setVoltage(0); // Stop intake if a gamepiece is present
            } else {
                intakeRollers.setVoltage(Constants.intakeInVoltage); // Run motor if no intake is present
            }
        } else if (state == PizzaboxState.SHOOT_PREP) {
            shooterTop.set(Constants.shooterPrepPower); // Turn shooter to idle speed
        } else if (state == PizzaboxState.SHOOT) {
            shooterTop.setVoltage(Constants.shooterShootVoltage); // Turn shooter to max Voltage

            // Check if shooter is ready to shoot
            if (shootEncoder1.getVelocity() > Constants.shooterMinShootSpeed
                    && shootEncoder2.getVelocity() > Constants.shooterMinShootSpeed) {
                intakeRollers.setVoltage(Constants.intakeInVoltage); // Run intake
            } else {
                intakeRollers.setVoltage(0); // Turn Intake Off
            }

        } else if (state == PizzaboxState.FAST_SHOOT) {
            shooterTop.setVoltage(Constants.shooterShootVoltage); // Turn shooter to max Voltage

            // Check if shooter is ready to shoot
            if (shootEncoder1.getVelocity() > Constants.shooterFastShootSpeed
                    && shootEncoder2.getVelocity() > Constants.shooterFastShootSpeed) {
                intakeRollers.setVoltage(Constants.intakeInVoltage); // Run intake
            } else {
                intakeRollers.setVoltage(0); // Turn Intake Off
            }

        } else if (state == PizzaboxState.REVERSE) {
            // Reverse shooter and intake
            shooterTop.setVoltage(-Constants.shooterRevVoltage * 0.5);
            intakeRollers.setVoltage(-Constants.intakeOutVoltage);
        } else {
            // Turn shooter and intake off
            shooterTop.setVoltage(0);
            intakeRollers.setVoltage(0);
        }

        updateUI();
    }

    private void updateUI() {
        sb_state.setString(state.name());
        sb_shooterTopVolt.setDouble(shooterTop.getBusVoltage() * shooterTop.getAppliedOutput());
        sb_shooterBotVolt.setDouble(shooterTop.getBusVoltage() * shooterTop.getAppliedOutput());
        sb_shooterTopRate.setDouble(shootEncoder1.getVelocity());
        sb_shooterBotRate.setDouble(shootEncoder2.getVelocity());
        sb_intakeRollerVolt.setDouble(intakeRollers.getMotorVoltage().getValueAsDouble());
        sb_intakeRollerRate.setDouble(intakeRollers.getVelocity().getValueAsDouble());
        sb_notePresent.setBoolean(isNotePresent());
        sb_intakeRollerCurrent.setDouble(intakeRollers.getStatorCurrent().getValueAsDouble());
    }

    /**
     * Static Initializer
     */
    public static IntakePizzaBox getInstance() {
        if (intake == null) {
            intake = new IntakePizzaBox();
        }
        return intake;
    }
}
