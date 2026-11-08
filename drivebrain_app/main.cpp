#include <FoxgloveServer.hpp>
#include <MCAPLogger.hpp>
#include <drivebrain_msgs.pb.h>
#include <FlowmeterComms.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>
#include <fmt/chrono.h>
#include <spdlog/spdlog.h>


// Atomic bools in cpp stop the race condition where two threads are trying to change the boolean at the same time
std::atomic<bool> running{true};

constexpr const char *FLOWMETER_CHIP = "/dev/gpiochip0";
constexpr uint32_t FLOWMETER_PIN = 12;

void sig_handler(int signal) {
    if(signal == SIGINT || signal == SIGTERM) {
      running = false;
    }
}

int main() {

  std::signal(SIGINT, sig_handler);
  std::signal(SIGTERM, sig_handler);

  // Creating a MCAP Logger
  // MCAP is a file structure that organizes sensor data, based on type and timestamp
  core::MCAPLogger::create(mcap::McapWriterOptions(""));
  
  // Creating a Foxglove server
  // Foxglove is a visualization tool for robotics and this data
  core::FoxgloveServer::create();

  // creating the file for the MCAP and initilize logging 
  core::MCAPLogger::instance().open_new_mcap();
  core::MCAPLogger::instance().init_logging();

  // Updating log
  spdlog::info("Constructed logging singletons");


  {
    core::ThreadSafeDeque<comms::Pulse> pulse_deque;
    comms::FlowmeterComms flowmeter_comms(FLOWMETER_CHIP, FLOWMETER_PIN, pulse_deque);
    comms::FlowmeterHandler flowmeter_handler(pulse_deque);

    if (flowmeter_comms.open_gpio() < 0) {
      spdlog::error("Could not open the flowmeter GPIO, exiting");
      core::MCAPLogger::instance().destroy();
      core::FoxgloveServer::instance().destroy();
      return 1;
    }

    // Start the consumer first
    flowmeter_handler.start();
    flowmeter_comms.start();

    auto loop_time = 0.004;
    std::chrono::microseconds loop_time_ms((int) (loop_time * 1000000.0f));
    auto next_tick = std::chrono::steady_clock::now();

    while(running) {

      next_tick += loop_time_ms;
      
  
      auto now = std::chrono::steady_clock::now();
      if(now > next_tick) {
        spdlog::warn("Loop overrun by {}", now-next_tick);
        next_tick = now;      
      }

      comms::FlowMeterRead reading = flowmeter_handler.get_latest();
      
      std::shared_ptr<drivebrain_msgs::FlowmeterData> msg = std::make_shared<drivebrain_msgs::FlowmeterData>();
      
      msg->set_pulse_count(reading.pulse_count); 
      msg->set_total_volume_l(reading.total_volume_l);

      core::MCAPLogger::instance().log_msg(msg);
      core::FoxgloveServer::instance().send_live_telem_msg(msg);

      std::this_thread::sleep_until(next_tick);
    }

    spdlog::warn("Interrupted, stopping mini-drivebrain");
  } // handler + comms threads joined here

  core::MCAPLogger::instance().destroy();
  core::FoxgloveServer::instance().destroy();
  spdlog::info("Logging singletons safely destroyed");

  return 0;
}
