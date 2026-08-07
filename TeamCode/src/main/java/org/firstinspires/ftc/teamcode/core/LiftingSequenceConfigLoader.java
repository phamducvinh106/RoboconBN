package org.firstinspires.ftc.teamcode.core;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public final class LiftingSequenceConfigLoader {
    private static final Pattern NUMBER=Pattern.compile("\\\"([A-Za-z0-9_]+)\\\"\\s*:\\s*(-?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+-]?\\d+)?)");
    private static final Pattern BOOLEAN=Pattern.compile("\\\"(endstopActiveLow|irActiveLow)\\\"\\s*:\\s*(true|false)");
    private static final String[] REQUIRED={"placeLeft","placeRight","holdLeft","holdRight","stepHighNs","stepLowNs","irDebounceNs","sensorStaleNs","elevatorTimeoutNs","stateTimeoutNs","homeSteps","ready1Steps","lift1Steps","ready2Steps","lift2Steps","maxRetries","settleCycles","releaseBackOutCm","positionToleranceCm","headingToleranceDeg","encoderFreshnessNs","noProgressCm","scanSpeed","centerSpeed","approachSpeed"};
    private LiftingSequenceConfigLoader() {}
    public static LiftingSequenceConfig load(String json) { if(json==null||json.trim().isEmpty()) throw new IllegalArgumentException("missing config"); if(!json.trim().startsWith("{")||!json.trim().endsWith("}")||json.chars().filter(c->c=='{').count()!=json.chars().filter(c->c=='}').count()) throw new IllegalArgumentException("malformed config");
        Matcher vm=Pattern.compile("\\\"version\\\"\\s*:\\s*(\\d+)").matcher(json); if(!vm.find()||Integer.parseInt(vm.group(1))!=LiftingSequenceConfig.SCHEMA_VERSION) throw new IllegalArgumentException("wrong config version");
        Map<String,Double> n=new LinkedHashMap<>(); Matcher m=NUMBER.matcher(json); while(m.find()) n.put(m.group(1),Double.valueOf(m.group(2))); for(String k:REQUIRED) if(!n.containsKey(k)||!Double.isFinite(n.get(k))) throw new IllegalArgumentException("missing or non-finite field: "+k);
        Map<String,Boolean> b=new LinkedHashMap<>(); Matcher bm=BOOLEAN.matcher(json); while(bm.find()) b.put(bm.group(1),Boolean.valueOf(bm.group(2))); if(b.size()!=2) throw new IllegalArgumentException("missing sensor polarity");
        range(n,"placeLeft",0,1); range(n,"placeRight",0,1); range(n,"holdLeft",0,1); range(n,"holdRight",0,1); range(n,"stepHighNs",1,1e9); range(n,"stepLowNs",1,1e9); range(n,"irDebounceNs",1,1e12); range(n,"sensorStaleNs",1,1e12); range(n,"elevatorTimeoutNs",1,1e13); range(n,"stateTimeoutNs",1,1e13); range(n,"homeSteps",0,0); range(n,"ready1Steps",1,1e6); range(n,"lift1Steps",1,1e6); range(n,"ready2Steps",1,1e6); range(n,"lift2Steps",1,1e6); range(n,"maxRetries",0,10); range(n,"settleCycles",1,100); range(n,"releaseBackOutCm",0.01,1000); range(n,"positionToleranceCm",0.01,100); range(n,"headingToleranceDeg",0.01,360); range(n,"encoderFreshnessNs",1,1e12); range(n,"noProgressCm",0.01,100); range(n,"scanSpeed",0.01,1); range(n,"centerSpeed",0.01,1); range(n,"approachSpeed",0.01,1);
        Map<String,LiftingSequenceConfig.Factory> f=new LinkedHashMap<>(); Matcher fm=Pattern.compile("\\\"(01|02|03|04)\\\"\\s*:\\s*\\{\\s*\\\"near\\\"\\s*:\\s*\\{\\s*\\\"x\\\"\\s*:\\s*(-?[0-9.]+)\\s*,\\s*\\\"y\\\"\\s*:\\s*(-?[0-9.]+)\\s*,\\s*\\\"heading\\\"\\s*:\\s*(-?[0-9.]+)\\s*\\}\\s*,\\s*\\\"placement\\\"\\s*:\\s*\\{\\s*\\\"x\\\"\\s*:\\s*(-?[0-9.]+)\\s*,\\s*\\\"y\\\"\\s*:\\s*(-?[0-9.]+)\\s*,\\s*\\\"heading\\\"\\s*:\\s*(-?[0-9.]+)").matcher(json); while(fm.find()) f.put(fm.group(1),new LiftingSequenceConfig.Factory(fm.group(1),new LiftingSequenceConfig.Pose(Double.parseDouble(fm.group(2)),Double.parseDouble(fm.group(3)),Double.parseDouble(fm.group(4))),new LiftingSequenceConfig.Pose(Double.parseDouble(fm.group(5)),Double.parseDouble(fm.group(6)),Double.parseDouble(fm.group(7))))); if(f.size()!=4) throw new IllegalArgumentException("missing factory calibration");
        return LiftingSequenceConfig.create(Integer.parseInt(vm.group(1)),n,b,f,fingerprint(json)); }
    private static void range(Map<String,Double> n,String k,double min,double max){double v=n.get(k); if(v<min||v>max||v!=Math.rint(v)&&k.endsWith("Steps")) throw new IllegalArgumentException("out of range: "+k);}
    private static String fingerprint(String s){try{byte[] h=MessageDigest.getInstance("SHA-256").digest(s.getBytes(StandardCharsets.UTF_8)); StringBuilder b=new StringBuilder(); for(int i=0;i<4;i++) b.append(String.format("%02x",h[i])); return b.toString();}catch(Exception e){throw new IllegalStateException(e);}}
}
