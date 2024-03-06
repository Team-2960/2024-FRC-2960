package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {

    private static Pneumatics instance = null;

    public Compressor compressor;

    private GenericEntry sb_pressure;
    private GenericEntry sb_current;

    private Pneumatics() {
        compressor = new Compressor(Constants.phCANID, PneumaticsModuleType.REVPH);
        compressor.enableAnalog(Constants.minPressure, Constants.maxPressure);

        var layout = Shuffleboard.getTab("Status")
                .getLayout("Pneumatics", BuiltInLayouts.kList)
                .withSize(1, 2);
        sb_pressure = layout.add("Pressure", 0).getEntry();
        sb_current = layout.add("Current", 0).getEntry();
    }

    @Override
    public void periodic() {
        updateUI();
    }

    public void updateUI() {
        sb_pressure.setDouble(compressor.getPressure());
        sb_current.setDouble(compressor.getCurrent());
    }

    public static Pneumatics getInstance() {
        if (instance == null)
            instance = new Pneumatics();

        return instance;
    }

}
