package frc.robot.elastic;

import org.json.simple.JSONObject;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;

public class Reef implements NTSendable{
    private boolean[][] coralPlaced;
    private boolean[] algaePlaced;
    public Reef(){
        coralPlaced=new boolean[3][12];
        algaePlaced=new boolean[6];
        for(int i=0; i<algaePlaced.length; i++){
            algaePlaced[i]=true;
        }
    }
    public boolean isCoralPlaced(int level,int branch){
        return coralPlaced[level-2][branch];
    }
    public void setCoralPlaced(int level,int branch,boolean set){
        coralPlaced[level-2][branch]=set;
    }
    public boolean isAlgaePlaced(int side){
        return algaePlaced[side];
    }
    public void setAlgaePlaced(int side,boolean set){
        algaePlaced[side]=set;
    }
    public boolean isCoralBlocked(int level,int branch){
        int side=branch/2;
        if(algaePlaced[side]){
            int lowAlgae=side%2;
            if(lowAlgae==0){
                return level==3;
            }else{
                return (level==2 || level==3);
            }
        }else{
            return false;
        }
    }
    @SuppressWarnings("unchecked")
    public String jsonify(){
        JSONObject obj = new JSONObject();
        for (int i=0; i<coralPlaced.length; i++){
            for (int j=0; j<coralPlaced[i].length; j++){
                String propName="Branch "+j+" L"+(i+2);
                if(isCoralBlocked(i+2,j)){
                    obj.put(propName, "Blocked");
                }else if(isCoralPlaced(i+2,j)){
                    obj.put(propName, "Placed");
                }else{
                    obj.put(propName, "Open");
                }
            }
        }
        return obj.toJSONString();
    }
    @Override
    public void initSendable(NTSendableBuilder builder) {
        for (int i=0; i<coralPlaced.length; i++){
            for (int j=0; j<coralPlaced[i].length; j++){
                int icopy=i;
                int jcopy=j;
                builder.addBooleanProperty("Branch "+j+" L"+(i+2), ()->isCoralPlaced(icopy+2, jcopy), (value)->setCoralPlaced(icopy+2, jcopy, value));
            }
        }
        for(int i=0; i<algaePlaced.length; i++){
            int icopy=i;
            builder.addBooleanProperty("Side "+i, ()->isAlgaePlaced(icopy), (value)->setAlgaePlaced(icopy, value));
        }
        builder.addStringProperty("ReefJson", ()->jsonify(), null);
        builder.setSmartDashboardType("Reef");
    }
}
