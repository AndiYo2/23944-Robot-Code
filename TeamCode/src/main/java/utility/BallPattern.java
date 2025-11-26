package utility;


public class BallPattern {
    private BallType[] pattern;


    public enum BallType {
        NONE,
        PURPLE,
        GREEN
    }
    public BallPattern(){
        setBallPattern(BallType.NONE,BallType.NONE,BallType.NONE);
    }


    public BallPattern(BallType ballType, BallType ballType1, BallType ballType2) {
        setBallPattern(ballType,ballType1,ballType2);

    }


    public void setBallPattern(BallType one, BallType two, BallType three){
        pattern = new BallType[]{one, two, three};
    }

    public void setBallPatternNone(int x){
        pattern[x -1 ] = BallType.NONE;
    }
    public void addBallInSlotX(int x, BallType ballType){
        pattern[x -1 ] = ballType;
    }


    public BallType getBallInSlotX(int x){
        return pattern[x - 1];
    }

    public BallType[] getBallPattern(){
        return pattern;
    }
}