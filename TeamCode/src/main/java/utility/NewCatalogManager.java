package utility;

import Constants.EnumConstants.CatalogingCases;
import Constants.SpindexerConstants;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import subsystems.Intake;
import subsystems.Spindexer;


public class NewCatalogManager {
    private final Intake intake;
    private final Telemetry telemetry;
    private final DualBallDetector sensors;
    private final Spindexer spindexer;
    private int indexed = 0;

    private CatalogingCases state = CatalogingCases.Idle;

    // Timeout management
    private ElapsedTime stateTimer = new ElapsedTime();


    public NewCatalogManager(Spindexer spindexer, Intake intake, Telemetry telemetry, DualBallDetector sensors) {
        this.spindexer = spindexer;
        this.telemetry = telemetry;
        this.intake = intake;
        this.sensors = sensors;
    }

    public CatalogingCases getState(){
        return state;
    }


    public void update() {
        sensors.update();
        switch (state) {
            case Idle:
                return;
            case Scanning:
                stateTimer.reset();
                //get slot0 color
                //get transfer color
                //get intake color

                //set the slots in the spindexer accordingly
                indexed = 1;
                state = CatalogingCases.Rotate;


                break;
            case Rotate:
                if(indexed < 3) {
                    indexed++;
                    spindexer.rotateCCW();
                    state = CatalogingCases.RotationWait;
                }
               break;
            case RotationWait:
                if(stateTimer.seconds() > SpindexerConstants.ROTATION_TIME) {
                    return;
                }
                state = CatalogingCases.MovingBall;
                stateTimer.reset();
                intake.runIntake();
                break;
            case MovingBall:
                if(stateTimer.seconds() > SpindexerConstants.INTAKE_TIME) {
                    return;
                }
                intake.stopIntake();
                state = CatalogingCases.Rotate;
                break;
        }
    }



    public void initiateCataloging(){
        if(state != CatalogingCases.Idle){
            return;
        }
        state = CatalogingCases.Scanning;
    }










}
