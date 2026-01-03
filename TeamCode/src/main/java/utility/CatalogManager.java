package utility;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.RobotConstants.Enums.CatalogingCases;

import static utility.SpindexerAndMotifStatus.SpindexerPattern.setBallInSlotX;

public class CatalogManager {
   private final Intake intake;
    private final Telemetry telemetry;
    private final DualBallDetector sensors;
    private final Spindexer spindexer;
    private int indexed = 0;

    private CatalogingCases state = RobotConstants.Enums.CatalogingCases.Idle;;


    public CatalogManager(Spindexer spindexer, Intake intake, Telemetry telemetry, DualBallDetector sensors) {
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
                DualBallDetector.Result result = sensors.detectBall();
                if (result.ballPresent) {
                    indexed++;
                    setBallInSlotX(0, result.color);
                    if(indexed == 3){
                        state = CatalogingCases.RotateToEndLocation;
                        break;
                    }else{
                        state = CatalogingCases.WaitingForRotation;
                        intake.runIntake();
                        spindexer.rotateCCW();
                    }
                }
                break;
            case WaitingForRotation:
                if(spindexer.isRotationIdle()){
                    intake.stopIntake();
                    state = CatalogingCases.Scanning;
                }
                break;
            case RotateToEndLocation:
                state = CatalogingCases.Idle;
                intake.stopIntake();
                indexed = 0;
                spindexer.rotateToColor(SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0));

        }
    }


    public void initiateCataloging(){
        if(state != CatalogingCases.Idle){
            return;
        }
        state = CatalogingCases.Scanning;
        indexed = 0;
    }










}
