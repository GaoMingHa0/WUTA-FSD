#include "can_interface/mode_cache.hpp"
#include <fstream>
#include <sstream>
#include <map>
#include <cctype>
#include <cmath>
#include <ctime>
#include <sys/stat.h>
#include <unistd.h>
namespace can_interface {
namespace {
std::string read(const std::string & path) {
  std::ifstream f(path); if (!f) return {};
  std::string out, line;
  while (std::getline(f,line)) {
    out += line + '\n'; if (out.size()>4096) return {};
  }
  return out;
}
uint64_t startTicks(const std::string & pid) {
  auto stat=read("/proc/"+pid+"/stat");
  auto end=stat.rfind(')'); if (end==std::string::npos) return 0;
  std::istringstream fields(stat.substr(end+2)); std::string value;
  for (int i=3;i<=22;++i) if (!(fields>>value)) return 0;
  return std::stoull(value);
}
}
bool readModeCache(const std::string & directory, const std::string & iface,
  uint32_t id, double max_age_sec, ModeSnapshot & snapshot, std::string & reason)
{
  try {
    if (iface.empty() || iface=="." || iface==".." || iface.size()>63) {
      reason="invalid interface"; return false;
    }
    for (unsigned char c:iface) if (!std::isalnum(c) && c!='_' && c!='-' && c!='.') {
      reason="invalid interface"; return false;
    }
    const auto path=directory+"/"+iface+".mode";
    struct stat st{};
    if (lstat(path.c_str(),&st)!=0 || !S_ISREG(st.st_mode) || st.st_size>4096) {
      reason="cache unavailable"; return false;
    }
    auto text=read(path); if (text.empty()) { reason="empty cache"; return false; }
    std::map<std::string,std::string> values;
    std::istringstream input(text); std::string line;
    while (std::getline(input,line)) {
      auto sep=line.find('=');
      if (sep==std::string::npos || !values.emplace(line.substr(0,sep),line.substr(sep+1)).second) {
        reason="invalid cache format"; return false;
      }
    }
    auto number=[&](const char *key) {
      const auto & v=values.at(key);
      if (v.empty() || v.find_first_not_of("0123456789")!=std::string::npos)
        throw std::invalid_argument("invalid number");
      return std::stoull(v);
    };
    auto boot=read("/proc/sys/kernel/random/boot_id");
    if (number("version")!=1 || values.at("boot_id")+"\n"!=boot ||
        values.at("iface")!=iface || number("can_id")!=id || number("valid")!=1 ||
        values.at("session_id").empty()) { reason="cache identity/validity mismatch"; return false; }
    const auto mode=number("mode"), seq=number("sequence"), pid=number("pid");
    const auto ticks=number("process_start_ticks"), stamp=number("received_monotonic_ns");
    (void)number("received_unix_ns");
    if (mode>255 || seq==0 || pid==0 || pid>2147483647 || ticks==0 || stamp==0) {
      reason="invalid cache value"; return false;
    }
    const auto pid_text=std::to_string(pid);
    if (startTicks(pid_text)!=ticks) { reason="bridge process unavailable"; return false; }
    // A zombie has already stopped serving CAN even if /proc still exists.
    auto stat=read("/proc/"+pid_text+"/stat"); auto end=stat.rfind(')');
    if (end==std::string::npos || stat.size()<end+3 || stat[end+2]=='Z' || stat[end+2]=='X') {
      reason="bridge process stopped"; return false;
    }
    auto comm=read("/proc/"+pid_text+"/comm");
    if (comm!="zlg_can_bridge\n") { reason="not a bridge process"; return false; }
    struct timespec now{}; clock_gettime(CLOCK_MONOTONIC,&now);
    const uint64_t current=uint64_t(now.tv_sec)*1000000000ULL+now.tv_nsec;
    if (stamp>current) { reason="future cache timestamp"; return false; }
    const double age=double(current-stamp)/1e9;
    if (!std::isfinite(max_age_sec) || (max_age_sec>0 && age>max_age_sec)) {
      reason="cache expired"; return false;
    }
    if (read(path)!=text || startTicks(pid_text)!=ticks) {
      reason="cache changed while reading"; return false;
    }
    snapshot={static_cast<uint8_t>(mode),seq,age,values.at("session_id")};
    reason.clear(); return true;
  } catch (const std::exception &) { reason="invalid or inaccessible cache"; return false; }
}
}
