package frc.robot.elastic;

import org.json.simple.JSONObject;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ReefScoring implements Sendable{
    private Reef reef;
    public ReefScoring(Reef reef){
        this.reef=reef;
    }
    public int getCount(int level){
        int count=0;
        for(int j=0; j<12; j++){
            if(reef.isCoralPlaced(level,j)){
                count++;
            }
        }
        return count;
    }
    @SuppressWarnings("unchecked")
    public String jsonify(){
        JSONObject obj = new JSONObject();
        // TODO add L1 scoring
        obj.put("L1",0);
        for (int i=2; i<=4; i++){
            obj.put("L"+i, getCount(i));
        }
        return obj.toJSONString();
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO add L1 scoring
        builder.addIntegerProperty("L1", ()->0, null);
        for(int i=2;i<=4;i++){
            int icopy=i;
            builder.addIntegerProperty("L"+i, ()->getCount(icopy), null);
        }
        builder.addStringProperty("ScoringJson", ()->jsonify(), null);
        builder.setSmartDashboardType("ReefScoring");
    }
    
}
