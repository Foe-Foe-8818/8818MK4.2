package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class AutonomousCommand extends SequentialCommandGroup {
    public AutonomousCommand(WPI_TalonSRX liftMotor) {
        addCommands(
            new L4elevator(liftMotor, 0.5, 6000) // Run motor at 50% speed for 6000 milliseconds (6 seconds)
        );
    }
}