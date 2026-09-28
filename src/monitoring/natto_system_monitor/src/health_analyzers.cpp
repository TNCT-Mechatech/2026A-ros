#include <diagnostic_aggregator/analyzer_group.hpp>
#include <diagnostic_aggregator/generic_analyzer.hpp>
#include <natto_system_monitor/health.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <type_traits>

namespace natto_system_monitor {
// Retain the upstream item matching, timeout handling and hierarchy. Only replace
// the parent summary: mixed OK/STALE must not become a fabricated ERROR.
template <class Base>
class HealthAnalyzerBase : public Base {
   public:
    std::vector<std::shared_ptr<Health>> report () override {
        auto          statuses = Base::report ();
        unsigned char level    = Health::OK;
        size_t        count    = 0;
        const auto    prefix   = this->getPath () + "/";
        for (const auto &s : statuses) {
            if (s->name.rfind (prefix, 0) != 0) continue;
            if constexpr (std::is_same_v<Base, diagnostic_aggregator::AnalyzerGroup>) {
                if (s->name.find ('/', prefix.size ()) != std::string::npos) continue;
            }
            level = combine (level, s->level);
            ++count;
        }
        for (auto &s : statuses)
            if (s->name == this->getPath ()) {
                s->level   = count ? level : Health::STALE;
                s->message = s->level == Health::OK ? "OK" : s->level == Health::WARN ? "WARN" : s->level == Health::ERROR ? "ERROR" : "STALE";
                diagnostic_msgs::msg::KeyValue state;
                state.key = "operational_state";
                // A category has no single robot operating mode; retain each leaf state.
                state.value = "UNKNOWN";
                s->values.push_back (state);
            }
        return statuses;
    }
};
class HealthAnalyzer : public HealthAnalyzerBase<diagnostic_aggregator::GenericAnalyzer> {};
class HealthGroup : public HealthAnalyzerBase<diagnostic_aggregator::AnalyzerGroup> {};
}  // namespace natto_system_monitor
PLUGINLIB_EXPORT_CLASS (natto_system_monitor::HealthAnalyzer, diagnostic_aggregator::Analyzer)
PLUGINLIB_EXPORT_CLASS (natto_system_monitor::HealthGroup, diagnostic_aggregator::Analyzer)
