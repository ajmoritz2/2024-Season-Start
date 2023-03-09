package frc.robot.auton.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class ArmWantedStateCommand extends InstantCommand {
    
    private final Arm arm;
    private final Arm.SystemState wantedState;

    public ArmWantedStateCommand(final Arm arm, Arm.SystemState wantedState){
        this.arm = arm;
        this.wantedState = wantedState;
    }

    @Override
    public void initialize(){
        arm.setWantedState(wantedState);
    }
}
