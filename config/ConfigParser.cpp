#include "ConfigParser.h"
#include <cerrno>
#include <climits>
#include <cfloat>
// Shared registry prevents parsing/echo drift.
#define INTS(X) \
 X("MICROSTEP_RATIO",microstep_ratio) \
 X("MAX_STEP_PER_CYCLE",max_step_per_cycle) \
 X("MAX_PHYSICAL_DELTA",max_physical_delta) \
 X("BACKLASH_STEPS",backlash_steps) \
 X("POS_MIN_LIMIT",pos_min_limit) \
 X("POS_MAX_LIMIT",pos_max_limit) \
 X("POS_CENTER",pos_center) \
 X("TOTAL_DURATION_MIN",total_duration_min) \
 X("FLUSH_INTERVAL",flush_interval) \
 X("BAUD_RATE",baud_rate)
#define FLOATS(X) \
 X("BASE_B0",b0) \
 X("CONTROL_FREQ_HZ",control_freq_hz) \
 X("TARGET_VPHS_MV",target_vphs_mv) \
 X("DEADBAND_MV",deadband_mv) \
 X("SWEEP_TOLERANCE_MV",sweep_tolerance_mv) \
 X("KP",kp) \
 X("KI",ki)
static void trim(char* s) {
    char* b=s;
    while (*b==' ' || *b=='\t' || *b=='\r' || *b=='\n') ++b;
    memmove(s,b,strlen(b)+1);
    size_t n=strlen(s);
    while(n && (s[n-1]==' ' || s[n-1]=='\t' || s[n-1]=='\r' || s[n-1]=='\n')) s[--n]=0;
}
static bool assign(SystemConfig& cfg,const char* key,const char* val) {
    char* end=nullptr; errno=0;
    double n=strtod(val,&end);
    if(errno || end==val || *end || !std::isfinite(n)) return false;
#define I(k,m) if(!strcmp(key,k)) { if(n<INT32_MIN || n>INT32_MAX || std::floor(n)!=n) return false; cfg.m=static_cast<int32_t>(n); return true; }
    INTS(I)
#undef I
#define F(k,m) if(!strcmp(key,k)) { if(std::fabs(n)>FLT_MAX) return false; cfg.m=static_cast<float>(n); return true; }
    FLOATS(F)
#undef F
    return false;
}
int ConfigParser_Load(const char* path,SystemConfig& cfg) {
    FILE* fp=fopen(path,"r"); if(!fp) { perror("ConfigParser"); return -1; }
    SystemConfig candidate;
    char line[CONFIG_MAX_LINE_LENGTH]; int number=0; bool ok=true;
    while(fgets(line,sizeof(line),fp)) {
        ++number;
        if(!strchr(line,'\n') && !feof(fp)) { ok=false; break; }
        char* p=strchr(line,'#'); if(p) *p=0;
        p=strstr(line,"//"); if(p) *p=0;
        trim(line); if(!*line) continue;
        char* eq=strchr(line,'='); if(!eq) { ok=false; break; }
        *eq++=0; trim(line); trim(eq);
        if(!assign(candidate,line,eq)) { ok=false; break; }
    }
    if(ferror(fp)) ok=false;
    fclose(fp);
    const char* why="malformed/unknown key or value";
    if(!ok || !candidate.Validate(&why)) {
        fprintf(stderr,"ConfigParser: rejected %s line %d: %s\n",path,number,why);
        return -1;
    }
    cfg=candidate;
    return 0;
}
void ConfigParser_Print(const SystemConfig& cfg) {
    printf("VERSION=%s\nCONTROLLER=PI\n",SOFTWARE_VERSION);
#define PRINT(k,m) printf(k "=%.12g\n",static_cast<double>(cfg.m));
    INTS(PRINT) FLOATS(PRINT)
#undef PRINT
    printf("EFFECTIVE_LOGICAL_LIMIT=%d\n",cfg.EffectiveLogicalLimit());
}
