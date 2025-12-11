//package utility;
//
//import java.util.ArrayList;
//
//
//public class OuttakeCases
//{
//    private BallPattern storedPattern;
//    private BallPattern goalPattern;
//
//
//    ArrayList<Integer> purpleBalls = new ArrayList<>();
//    ArrayList<Integer> greenBalls = new ArrayList<>();
//
//
//
//    public OuttakeCases( BallPattern goalPattern){
//        this.goalPattern = goalPattern;
//    }
//
//    public String calcShootingOrder(int top){
//
//        int size = purpleBalls.size()+ greenBalls.size();
//        int x =1;
//        String comp = "";
//
//        while(x <= size)
//
//            if(goalPattern.getBallInSlotX(x) != storedPattern.getBallInSlotX(top)){
//
//                //find slot the next color is located in
//                //rotate to that slot
//                if(goalPattern.getBallInSlotX(x) == BallPattern.BallType.PURPLE && !purpleBalls.isEmpty()){
//                    //rotate to:
//                    top = purpleBalls.get(0);
//                    comp += "rp ";
//
//                } else if (goalPattern.getBallInSlotX(x) == BallPattern.BallType.GREEN && !greenBalls.isEmpty()) {
//                    //rotate to:
//                    top = greenBalls.get(0);
//                    comp += "rg ";
//
//                }else{
//
//                    break;
//                }
//
//
//            }else{
//                comp += "" + storedPattern.getBallInSlotX(top) + " ";
//                storedPattern.setBallPatternNone(top);
//                removeColorCatagory(top);
//                x++;
//
//            }
//        return comp;
//    }
//
//    public void removeColorCatagory(int x){
//
//        if(purpleBalls.contains(x)){
//            purpleBalls.remove(Integer.valueOf(x));
//        }else{
//            greenBalls.remove(Integer.valueOf(x));
//        }
//
//    }
//
//    public void clearForTesting(){
//        purpleBalls.clear();
//        greenBalls.clear();
//    }
//
//
//    public void setBalls(BallPattern.BallType one,BallPattern.BallType two,BallPattern.BallType three ){
//
//        storedPattern = new BallPattern(one,two,three);
//
//        if(one == BallPattern.BallType.PURPLE){
//            purpleBalls.add(1);
//        }else if(one == BallPattern.BallType.GREEN){
//            greenBalls.add(1);
//        }
//        if(two == BallPattern.BallType.PURPLE){
//            purpleBalls.add(2);
//        }else if(two == BallPattern.BallType.GREEN){
//            greenBalls.add(2);
//        }
//        if(three == BallPattern.BallType.PURPLE){
//            purpleBalls.add(3);
//        }else if (three == BallPattern.BallType.GREEN){
//            greenBalls.add(3);
//        }else{
//            //Maybe not needed
//        }
//
//    }
//
//}
