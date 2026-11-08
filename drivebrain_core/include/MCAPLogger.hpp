#pragma once

#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cassert>
#include <queue>
#include <chrono>
#include <mcap/writer.hpp>
#include <google/protobuf/descriptor.pb.h>
#include <spdlog/spdlog.h>

namespace core {

    struct RawMessage_s {
        std::string serialized_data;
        std::string message_name; // topic
        std::string type_name;
        mcap::Timestamp log_time;
    };

    using MsgType = std::shared_ptr<google::protobuf::Message>;

    class MCAPLogger {
        public:

            /**
             * Constructor for initializing a new MCAPLogger singleton instance
             *
             * @param options options to create the mcap with
             */
            static void create(const mcap::McapWriterOptions &options);

            /**
             * Fetches MCAPLogger singleton instance
             *
             * @return MCAPLogger instance
             */
            static MCAPLogger& instance();

            /**
             * Destroys the MCAPLogger singleton instance, stopping logging and freeing resources
             */
            static void destroy();

            /**
             * Destructs MCAPLogger instance, frees singleton instance and joins all running log threads
             */
            ~MCAPLogger() {
              spdlog::info("Destructing mcap logger");
              {
                std::lock_guard lock(_input_buffer_mutex);
                _logging = false;
              }

              _s_instance.store(nullptr, std::memory_order_release);
              spdlog::info("Msg logger singleton instance released");

              _input_buffer_cv.notify_all();
              if(_msg_log_thread.joinable()) _msg_log_thread.join();
            }

            /**
             * Opens a new mcap file by adding options and all protobuf schema
             *
             * @return 0 on success, negative err code on failure
             */
            int open_new_mcap();

            /**
             * Kicks off the message logging thread and begins writing to a log file.
            */
            void init_logging();

            /**
             * Logs a protobuf message to the current MCAP file
             *
             * @param message the message to be logged
             * @param topic topic to log on, defaults to the message type name. A new topic gets its channel on first use.
             * @return 0 on success, negative err code on failure
             */
            int log_msg(MsgType message, const std::string &topic = "");

        private:

            /* Private constructor to be called by init method */
            MCAPLogger(const mcap::McapWriterOptions &options);

            /**
             * Spawned by thread, loops until end of program life or error occurs.
             * Logs all queued messages to a file.
             */
            void _handle_log_to_file();

            /* Singleton move semantics */
            MCAPLogger(const MCAPLogger&) = delete;
            MCAPLogger& operator=(const MCAPLogger&) = delete;

            /* Singleton instance */
            inline static std::atomic<MCAPLogger*> _s_instance;

            /* Message log management */
            std::deque<RawMessage_s> _input_buffer;
            std::mutex _input_buffer_mutex;
            std::condition_variable _input_buffer_cv;
            std::thread _msg_log_thread;

            /* MCAP utilities */
            mcap::McapWriter _writer;
            mcap::McapWriterOptions _options;

            /* State */
            std::unordered_map<std::string, uint32_t> _name_to_id_map;
            std::unordered_map<std::string, uint16_t> _type_to_schema_id_map;
            std::string _log_name = "NONE";
            bool _logging = false;
    };

}
