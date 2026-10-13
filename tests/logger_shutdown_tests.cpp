#include "../utils/LoggerShutdown.h"
#include <cstdio>
#include <string>

#define CHECK(condition) do { if(!(condition)) return __LINE__; } while(0)

struct FakeLogger {
    bool healthy=true;
    bool fail_on_close=false;
    bool closed=false;
    void Close() { closed=true; if(fail_on_close) healthy=false; }
    bool Healthy() const { return healthy; }
};

int main() {
    FILE* errors=std::tmpfile();
    CHECK(errors);
    FakeLogger good;
    bool failed=false;
    good.Close();
    if(!good.Healthy()) ReportLoggerShutdownFailure(failed,errors);
    CHECK(good.closed && !failed);

    FakeLogger late_failure;
    late_failure.fail_on_close=true;
    late_failure.Close();
    if(!late_failure.Healthy()) ReportLoggerShutdownFailure(failed,errors);
    CHECK(late_failure.closed && failed);
    std::rewind(errors);
    char text[256]={};
    CHECK(std::fgets(text,sizeof(text),errors));
    CHECK(std::string(text).find("logging shutdown failed")!=std::string::npos);
    std::fclose(errors);
}
