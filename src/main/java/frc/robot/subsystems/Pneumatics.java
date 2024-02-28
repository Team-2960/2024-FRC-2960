package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;


public class Pneumatics extends SubsystemBase {

    private static Pneumatics instance = null;

    public Compressor compressor;

    private Pneumatics() {
        compressor = new Compressor(/* PH CAN ID*/, PneumaticsModuleType.REVPH);
    }

    public static Pneumatics getInstance() {
        if(instance == null) instance = new Pneumatics();

        return instance;
    }

}
    