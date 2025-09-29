package org.firstinspires.ftc.teamcode.Outtake;

import org.firstinspires.ftc.teamcode.Globals.BallPattern;


public class OuttakeCases
{
    private BallPattern pattern = new BallPattern();
    private BallPattern PPG = new BallPattern("P","P","G");
    private BallPattern PGP = new BallPattern("P","G","P");
    private BallPattern GPP = new BallPattern("G","P","P");


    public OuttakeCases(BallPattern pattern){
        this.pattern = pattern;
    }

    public void calcShootingOrder(){
        if(pattern.equals(PPG)){

            if(pattern.getBallInSlotX(1).equals(PPG.getBallInSlotX(1))){
                if(pattern.getBallInSlotX(2).equals(PPG.getBallInSlotX(2))) {
                    System.out.println("PATTERN 1");
                }else{
                    System.out.println("PATTERN 3");
                }
            }else{
                System.out.println("PATTERN 2");
            }



        }else if(pattern.equals(PGP)){

        } else if (pattern.equals(GPP)) {

        }else{
            System.out.println("SOMEONE MESSED UP!!!!!!!! THEY DIDN'T MAKE A PATTERN!!!!!");
        }


    }

    public int getTopIndex(){
        return 0;
        //Find the index of the top center slot, possible fix / change if need be for offset
    }








}
