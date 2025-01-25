package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color8Bit;

public class MechanismConstants {
    public static final double mechWidth = 20;
    public static final double mechHeight = 50;
    // Realistic arm length
    public static final double armLength = (ArmConstants.armLength/ElevatorConstants.maxHeight)*mechHeight/2;
    public static final Color8Bit elevatorColor = new Color8Bit(255, 127, 0);
    public static final Color8Bit armColor = new Color8Bit(255, 0, 0);
}
