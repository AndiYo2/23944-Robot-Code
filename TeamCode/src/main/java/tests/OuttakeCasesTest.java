package tests;

import utility.BallPattern;
import utility.OuttakeCases;

public class OuttakeCasesTest {
    static int count = 0;

    public static void main(String[] args) {



        //PURPLE GREEN PURPLE TESTS
        System.out.println("PURPLE GREEN PURPLE TEST CASES:\n");
        OuttakeCases PGP = new OuttakeCases(new BallPattern(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE));
        String output;

        //TEST 1
        PGP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE);
        output = PGP.calcShootingOrder(1);
        System.out.println(output);
        addCount(output.equals("PURPLE rg GREEN rp PURPLE "));
        //TEST 2
        PGP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE);
        output = PGP.calcShootingOrder(2);
        System.out.println(output);
        addCount(output.equals("rp PURPLE rg GREEN rp PURPLE "));
        //TEST 3
        PGP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE);
        output = PGP.calcShootingOrder(3);
        System.out.println(output);
        addCount(output.equals("PURPLE rg GREEN rp PURPLE "));

        //TEST 4
        PGP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.NONE);
        output = PGP.calcShootingOrder(1);
        System.out.println(output);
        addCount(output.equals("PURPLE rg GREEN "));
        PGP.clearForTesting();
        //TEST 5
        PGP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.NONE);
        output = PGP.calcShootingOrder(2);
        System.out.println(output);
        addCount(output.equals("rp PURPLE rg GREEN "));
        PGP.clearForTesting();
        //TEST 6
        PGP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.NONE);
        output = PGP.calcShootingOrder(3);
        System.out.println(output);
        addCount(output.equals("rp PURPLE rg GREEN "));
        PGP.clearForTesting();

        //TEST 7
        PGP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.NONE);
        output = PGP.calcShootingOrder(1);
        System.out.println(output);
        addCount(output.equals("PURPLE "));
        PGP.clearForTesting();
        //TEST 8
        PGP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.NONE);
        output = PGP.calcShootingOrder(2);
        System.out.println(output);
        addCount(output.equals("rp PURPLE "));
        PGP.clearForTesting();
        //TEST 9
        PGP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.NONE);
        output = PGP.calcShootingOrder(3);
        System.out.println(output);
        addCount(output.equals("rp PURPLE "));
        PGP.clearForTesting();

        //TEST 10
        PGP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.PURPLE);
        output = PGP.calcShootingOrder(1);
        System.out.println(output);
        addCount(output.equals("PURPLE "));
        PGP.clearForTesting();
        //TEST 11
        PGP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.PURPLE);
        output = PGP.calcShootingOrder(2);
        System.out.println(output);
        addCount(output.equals("rp PURPLE "));
        PGP.clearForTesting();
        //TEST 12
        PGP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.PURPLE);
        output = PGP.calcShootingOrder(3);
        System.out.println(output);
        addCount(output.equals("PURPLE "));
        PGP.clearForTesting();

        //TEST 13
        PGP.setBalls(BallPattern.BallType.GREEN, BallPattern.BallType.NONE, BallPattern.BallType.NONE);
        output = PGP.calcShootingOrder(1);
        System.out.println(output+ "no result");
        addCount(output.equals(""));
        PGP.clearForTesting();






        System.out.println("\n\n\nPURPLE PURPLE GREEN TEST CASES:\n");
        //PURPLE PURPLE  GREEN TESTS
        OuttakeCases PPG = new OuttakeCases(new BallPattern(BallPattern.BallType.PURPLE, BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN));
        //TEST 1
        PPG.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE);
        output = PPG.calcShootingOrder(1);
        System.out.println(output);
        addCount(output.equals("PURPLE rp PURPLE rg GREEN "));
        //TEST 2
        PPG.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE);
        output = PPG.calcShootingOrder(2);
        System.out.println(output);
        addCount(output.equals("rp PURPLE rp PURPLE rg GREEN "));
        //TEST 3
        PPG.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE);
        output = PPG.calcShootingOrder(3);
        System.out.println(output);
        addCount(output.equals("PURPLE rp PURPLE rg GREEN "));

        //TEST 4
        PPG.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.NONE);
        output = PPG.calcShootingOrder(1);
        System.out.println(output);
        addCount(output.equals("PURPLE "));
        PPG.clearForTesting();
        //TEST 5
        PPG.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.NONE);
        output = PPG.calcShootingOrder(2);
        System.out.println(output);
        addCount(output.equals("rp PURPLE "));
        PPG.clearForTesting();
        //TEST 6
        PPG.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.NONE);
        output = PPG.calcShootingOrder(3);
        System.out.println(output);
        addCount(output.equals("rp PURPLE "));
        PPG.clearForTesting();

        //TEST 7
        PPG.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.NONE);
        output = PPG.calcShootingOrder(1);
        System.out.println(output);
        addCount(output.equals("PURPLE "));
        PPG.clearForTesting();
        //TEST 8
        PPG.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.NONE);
        output = PPG.calcShootingOrder(2);
        System.out.println(output);
        addCount(output.equals("rp PURPLE "));
        PPG.clearForTesting();
        //TEST 9
        PPG.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.NONE);
        output = PPG.calcShootingOrder(3);
        System.out.println(output);
        addCount(output.equals("rp PURPLE "));
        PPG.clearForTesting();

        //TEST 10
        PPG.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.PURPLE);
        output = PPG.calcShootingOrder(1);
        System.out.println(output);
        addCount(output.equals("PURPLE rp PURPLE "));
        PPG.clearForTesting();
        //TEST 11
        PPG.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.PURPLE);
        output = PPG.calcShootingOrder(2);
        System.out.println(output);
        addCount(output.equals("rp PURPLE rp PURPLE "));
        PPG.clearForTesting();
        //TEST 12
        PPG.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.PURPLE);
        output = PPG.calcShootingOrder(3);
        System.out.println(output);
        addCount(output.equals("PURPLE rp PURPLE "));
        PPG.clearForTesting();

        //TEST 13
        PPG.setBalls(BallPattern.BallType.NONE, BallPattern.BallType.GREEN, BallPattern.BallType.NONE);
        output = PPG.calcShootingOrder(1);
        System.out.println(output + "no result");
        addCount(output.equals(""));
        PPG.clearForTesting();






        System.out.println("\n\n\nGREEN PURPLE PURPLE TEST CASES:\n");
        //GREEN PURPLE PURPLE TESTS
        OuttakeCases GPP = new OuttakeCases(new BallPattern(BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE, BallPattern.BallType.PURPLE));
        //TEST 1
        GPP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE);
        output = GPP.calcShootingOrder(1);
        System.out.println(output);
        addCount(output.equals("rg GREEN rp PURPLE rp PURPLE "));
        //TEST 2
        GPP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE);
        output = GPP.calcShootingOrder(2);
        System.out.println(output);
        addCount(output.equals("GREEN rp PURPLE rp PURPLE "));
        //TEST 3
        GPP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE);
        output = GPP.calcShootingOrder(3);
        System.out.println(output);
        addCount(output.equals("rg GREEN rp PURPLE rp PURPLE "));

        //TEST 4
        GPP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.NONE);
        output = GPP.calcShootingOrder(1);
        System.out.println(output);
        addCount(output.equals("rg GREEN rp PURPLE "));
        GPP.clearForTesting();
        //TEST 5
        GPP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.NONE);
        output = GPP.calcShootingOrder(2);
        System.out.println(output);
        addCount(output.equals("GREEN rp PURPLE "));
        GPP.clearForTesting();
        //TEST 6
        GPP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.GREEN, BallPattern.BallType.NONE);
        output = GPP.calcShootingOrder(3);
        System.out.println(output);
        addCount(output.equals("rg GREEN rp PURPLE "));
        GPP.clearForTesting();

        //TEST 7
        GPP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.NONE);
        output = GPP.calcShootingOrder(1);
        System.out.println(output + "no result");
        addCount(output.equals(""));
        GPP.clearForTesting();
        //TEST 8
        GPP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.NONE);
        output = GPP.calcShootingOrder(2);
        System.out.println(output + "no result");
        addCount(output.equals(""));
        GPP.clearForTesting();
        //TEST 9
        GPP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.NONE);
        output = GPP.calcShootingOrder(3);
        System.out.println(output + "no result");
        addCount(output.equals(""));
        GPP.clearForTesting();

        //TEST 10
        GPP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.PURPLE);
        output = GPP.calcShootingOrder(1);
        System.out.println(output + "no result");
        addCount(output.equals(""));
        GPP.clearForTesting();
        //TEST 11
        GPP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.PURPLE);
        output = GPP.calcShootingOrder(2);
        System.out.println(output + "no result");
        addCount(output.equals(""));
        GPP.clearForTesting();
        //TEST 12
        GPP.setBalls(BallPattern.BallType.PURPLE, BallPattern.BallType.NONE, BallPattern.BallType.PURPLE);
        output = GPP.calcShootingOrder(3);
        System.out.println(output + "no result");
        addCount(output.equals(""));
        GPP.clearForTesting();

        //TEST 13
        GPP.setBalls(BallPattern.BallType.NONE, BallPattern.BallType.GREEN, BallPattern.BallType.NONE);
        output = GPP.calcShootingOrder(1);
        System.out.println(output);
        addCount(output.equals("rg GREEN "));
        GPP.clearForTesting();



        System.out.println("\n\n\nTOTAL TESTS PASSED: " + count + "/" + 39);

    }

    public static void addCount(boolean b){

        if(b){
            System.out.println("true");
            count++;
        }else{
            System.out.println("false");
        }


    }

}

