package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.finalizedSubsystems.shooterSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class shooterOff extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final shooterSubsystem shooterSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     **/

    public shooterOff(shooterSubsystem subsystem) {
        shooterSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    public double p = 11, i = 0, d = 0, f = 0.8;

    public PIDFController shooterController = new PIDFController(p, i, d, f);

    public double ticksPerDegree = 4.233;
    double target = 0;


    @Override
    public void initialize(){
        shooterSubsystem.shooterOff();
    }


    @Override
    public boolean isFinished(){
        return true;
    }


}
