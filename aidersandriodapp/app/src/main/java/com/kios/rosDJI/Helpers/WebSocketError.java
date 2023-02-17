package com.kios.rosDJI.Helpers;

public class WebSocketError  {
    public  enum Error {

         NORMAL(1000,"Close normal"),
        GOING_AWAY(1001,"Client is leaving"),
        PROTOCOL_ERROR(1002,"Endpoint received a malformed frame"),
        REFUSE(1003,"Endpoint received an unsupported frame (e.g. binary-only endpoint received text frame)\n"),
        NOCODE(1005,"Expected close status, received none"),
        ABNORMAL_CLOSE(1006,"No close code frame has been received"),
        NO_UTF8(1007,"Endpoint received inconsistent message (e.g. malformed UTF-8)"),
        POLICY_VALIDATION(1008,"Policy violation"),
        TOOBIG(1009,"Endpoint won't process large frame"),
        EXTENSION(1010,"Client wanted an extension which server did not negotiate"),
        UNEXPECTED_CONDITION(1011,"Internal server error while operating"),
        TLS_ERROR(1015,"Transport Layer Security handshake failure\n"),
        NEVERCONNECTED(-1,"NEVER CONNECTED"),
        BUGGYCLOSE(-2,"BUGGYCLOSE"),
        FLASHPOLICY(-3,"FLASHPOLICY"),
        UNKNOWN(-5,"UNKNOWN");


        public int code;public String reason;
         Error(int code,String reason){
            this.code=code;
            this.reason=reason;
        }
        @Override
        public  String toString(){


            return reason+" ("+code+")";
        }


    }
    public static Error getDescription(int code){

         Error error[]=Error.values();

         for (Error e:error){
             if (e.code==code){
                 return  e;
             }

         }
         return Error.UNKNOWN;
    }


}
