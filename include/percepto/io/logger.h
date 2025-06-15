#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <memory>

inline std::shared_ptr<spdlog::logger> get_percepto_logger()
{
  auto logger = spdlog::get("percepto");
  if (logger)
  {
    return logger;
  }

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::trace);

  logger = std::make_shared<spdlog::logger>("percepto", console_sink);
  logger->set_level(spdlog::level::debug);
  logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
  spdlog::register_logger(logger);

  return logger;
}
