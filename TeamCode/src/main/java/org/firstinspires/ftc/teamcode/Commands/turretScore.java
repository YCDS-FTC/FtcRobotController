package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.finalizedSubsystems.turretSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class turretScore extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final turretSubsystem turretSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     **/

    public turretScore(turretSubsystem subsystem) {
        turretSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    public double p = 0.025, i = 0, d = 0.0004, f = 0;

    public PIDFController turretController = new PIDFController(p, i, d, f);

    public double ticksPerDegree = 4.233;
    double target = 135;


    @Override
    public void initialize(){
        turretController.setPIDF(p,i,d,f);
    }


    @Override
    public void execute(){

        double turretAngle = turretSubsystem.getTurretPosition();

        double output  = turretController.calculate(turretAngle, target);

        turretSubsystem.setTurretPower(output);

    }


    @Override
    public boolean isFinished(){

       return false;


    }


}
