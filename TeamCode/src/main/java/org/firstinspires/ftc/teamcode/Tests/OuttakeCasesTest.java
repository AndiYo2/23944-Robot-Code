package org.firstinspires.ftc.teamcode.Tests;

import org.firstinspires.ftc.teamcode.Globals.BallPattern;
import org.firstinspires.ftc.teamcode.Outtake.OuttakeCases;

public class OuttakeCasesTest {

    public static void main(String[] args) {

        OuttakeCases test = new OuttakeCases(new BallPattern(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE));
        test.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE);
        test.calcShootingOrder(1);


    }

}
