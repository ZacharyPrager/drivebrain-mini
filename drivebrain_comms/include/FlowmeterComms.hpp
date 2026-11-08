#pragma once
#include <linux/gpio.h>
#include <cstdint>
#include <atomic>
#include <string>
#include <thread>
#include <DriverBus.hpp>
#include <mutex>

namespace comms {

    struct Pulse {
        uint64_t timestamp_ns; // When the pulse happened
        uint32_t seqno; // If there is any missed pulses
    };

    struct FlowMeterRead {
        uint64_t pulse_count = 0;
        float total_volume_l = 0.0f;
    };

    class FlowmeterComms {
        public:
            /**
            * Constructor for a Flowmeter producer
            * Only stores the args, does nothing that can fail (like MCAPLogger)
            * Call open_gpio() then start() after constructing
            * @param chip_path GPIO chip device file
            * @param pin the GPIO pin the flowmeter will send Pulses to
            * @param deque  threadSafe deque for the Pulse data
            */
            FlowmeterComms(const std::string &chip_path, uint32_t pin, core::ThreadSafeDeque<Pulse> &deque);

            /**
            * Opens the GPIO chip at _chip_path
            * Requests _pin as an input that detects rising edges
            * Saves the line's fd into _line_fd and closes the chip fd
            * @return 0 on success, -1 on failure (like open_new_mcap)
            */
            int open_gpio();

            /**
            * Launches _thread to run read_pulses()
            * Only call after open_gpio() succeeded (_line_fd is valid)
            */
            void start();

            /**
            * Destructor that safely stops the producer thread
            * log that its been deconstructed
            * Should switch _running to false to request the thread to stop
            * Wait for the thread to finish
            * Close the fd on the GPIO pin
            */
            ~FlowmeterComms();

        private:

            /**
            * Runs on a _thread, reading from GPIO waiting for data (edges)
            * Package the data into a Pulse message
            * Pushes the Pulse into the _deque until _running is false
            * Notify the consumer new data has been produced
            */ 
            void read_pulses();
            
            std::atomic<bool> _running{true}; // Thread keeps running while this is true
            std::thread _thread; // The actual thread that listens to the GPIO
            int _line_fd = -1; // the GPIO pin to listen to, defaults to -1
            core::ThreadSafeDeque<Pulse> &_deque; // Where to send the data

            std::string _chip_path;
            uint32_t _pin;
     


        };

    class FlowmeterHandler {
        public:
            /**
            * Constructor for the Flowmeter consumer
            * @param deque the same ThreadSafeDeque
            */
            FlowmeterHandler(core::ThreadSafeDeque<Pulse> &deque);

            /**
            * Launches _thread to run handle_pulses()
            */
            void start();

            /**
            * Called by main() each tick, returns a COPY of _latest under _latest_mutex
            */
            const FlowMeterRead get_latest();
            /**
            * Destructor that safely stops the consumer thread
            */
            ~FlowmeterHandler();

        private:

            /**
            * Runs on _thread, waits on the deque's cv for Pulses
            */
            void handle_pulses();

            std::atomic<bool> _running{true}; 
            std::thread _thread; 
            FlowMeterRead _latest; 
            core::ThreadSafeDeque<Pulse> &_deque; 
            std::mutex _latest_mutex;

            static constexpr float PULSES_PER_LITER = 460.0f;
            uint32_t _last_seqno = 0;
            
        };

}