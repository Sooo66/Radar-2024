#include <csignal>

#include "srm/core.hpp"

void SignalHandler(int) {
  LOG(WARNING) << "Caught interrupt signal. Attempting to exit...";
  srm::core::BaseCore::exit_signal = true;
}

int main(int arc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = FLAGS_colorlogtostderr = FLAGS_log_prefix = true;
  FLAGS_log_dir = "../log/";
  srm::cfg.Parse("../config.toml");
  std::unique_ptr<srm::core::BaseCore> core;
  auto mode = srm::cfg.Get<std::string>({"mode"});
  core.reset(srm::core::CreateCore(mode));
  if (!core) {
    LOG(ERROR) << "Failed to create " << mode << " core";
    return -1;
  }
  if (!core->Initialize()) {
    LOG(ERROR) << "Failed to initialize core.";
    return 1;
  }
  std::signal(SIGINT, &SignalHandler);
  std::signal(SIGTERM, &SignalHandler);
  int ret = core->Run();
  google::ShutdownGoogleLogging();
  return ret;
}