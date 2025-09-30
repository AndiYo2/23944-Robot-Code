package org.firstinspires.ftc.teamcode.Outtake;

import org.firstinspires.ftc.teamcode.Globals.BallPattern;
import java.util.ArrayList;


public class OuttakeCases
{
    private BallPattern storedPattern;
    private BallPattern goalPattern;


    ArrayList<Integer> purpleBalls = new ArrayList<>();
    ArrayList<Integer> greenBalls = new ArrayList<>();



    public OuttakeCases( BallPattern goalPattern){
        this.goalPattern = goalPattern;
    }

    public void calcShootingOrder(int top){

        int size = purpleBalls.size()+ greenBalls.size();
        int x =1;

        while(x <= size)

            if(goalPattern.getBallInSlotX(top) != storedPattern.getBallInSlotX(top)){

                //find slot the next color is located in
                //rotate to that slot
                if(goalPattern.getBallInSlotX(x) == BallPattern.BallType.PURPLE && !purpleBalls.isEmpty()){
                    //rotate to:
                    top = purpleBalls.get(0);
                    purpleBalls.remove(0);
                    System.out.println("rotated to a purple");

                } else if (goalPattern.getBallInSlotX(x) == BallPattern.BallType.GREEN && !greenBalls.isEmpty()) {
                    //rotate to:
                    top = greenBalls.get(0);
                    greenBalls.remove(0);
                    System.out.println("rotated to a green");

                }else{
                    break;
                }


            }else{
                System.out.println(x + " - " + storedPattern.getBallInSlotX(top));
                storedPattern.setBallPatternNone(top);
                x++;

            }

    }



    public void setBalls(BallPattern.BallType one,BallPattern.BallType two,BallPattern.BallType three ){

        storedPattern = new BallPattern(one,two,three);

        if(one == BallPattern.BallType.PURPLE){
            purpleBalls.add(1);
        }else{
            greenBalls.add(1);
        }
        if(two == BallPattern.BallType.PURPLE){
            purpleBalls.add(2);
        }else{
            greenBalls.add(2);
        }
        if(three == BallPattern.BallType.PURPLE){
            purpleBalls.add(3);
        }else{
            greenBalls.add(3);
        }

    }

}