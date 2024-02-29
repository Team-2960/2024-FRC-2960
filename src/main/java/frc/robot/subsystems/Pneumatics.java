package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


public class Pneumatics extends SubsystemBase {

    private static Pneumatics instance = null;

    public Compressor compressor;

    private Pneumatics() {
        compressor = new Compressor(20,PneumaticsModuleType.REVPH);
    }

    public static Pneumatics getInstance() {
        if(instance == null) instance = new Pneumatics();

        return instance;
    }

}
    