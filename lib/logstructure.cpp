
#include "log.h"
#include "AP_Progmem.h"
//#include "util.h"
#include "defines.h"


const struct LogStructure log_structure[] PROGMEM = {
    { LOG_FORMAT_MSG, sizeof(log_Format), 
      "FMT", "BBnNZ",      "Type,Length,Name,Format" }, 
    { LOG_GPS_MSG, sizeof(log_GPS),
      "GPS",  "BIHBcLLeeEefI", "Status,TimeMS,Week,NSats,HDop,Lat,Lng,RelAlt,Alt,Spd,GCrs,VZ,T" },
    { LOG_IMU_MSG, sizeof(log_IMU),
      "IMU",  "Iffffff",     "TimeMS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ" },
    { LOG_MESSAGE_MSG, sizeof(log_Message),
      "MSG",  "Z",     "Message"},
    { LOG_RCIN_MSG, sizeof(log_RCIN),
      "RCIN",  "Ihhhhhhhh",     "TimeMS,Chan1,Chan2,Chan3,Chan4,Chan5,Chan6,Chan7,Chan8" },
    { LOG_RCOUT_MSG, sizeof(log_RCOUT),
      "RCOU",  "Ihhhhhhhh",     "TimeMS,Chan1,Chan2,Chan3,Chan4,Chan5,Chan6,Chan7,Chan8" },
    { LOG_BARO_MSG, sizeof(log_BARO),
      "BARO",  "Iffc",     "TimeMS,Alt,Press,Temp" },
    { LOG_CURRENT_MSG, sizeof(log_Current),             
      "CURR", "IhIhhhf",     "TimeMS,ThrOut,ThrInt,Volt,Curr,Vcc,CurrTot" },
    { LOG_COMPASS_MSG, sizeof(log_Compass),             
      "MAG", "Ihhhhhhhhh",    "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ,MOfsX,MOfsY,MOfsZ" },
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "BBHHIhBHB",    "RenCnt,RenBlw,NLon,NLoop,MaxT,PMT,I2CErr,INSErr,INAVErr" },
    { LOG_ATTITUDE_MSG, sizeof(log_Attitude),       
      "ATT", "IccccCC",      "TimeMS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "",            "" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "Bh",         "Id,Value" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "BH",         "Id,Value" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "Bi",         "Id,Value" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "BI",         "Id,Value" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "Bf",         "Id,Value" },
    { LOG_ERROR_MSG, sizeof(log_Error),         
      "ERR",   "BB",         "Subsys,ECode" },
};
