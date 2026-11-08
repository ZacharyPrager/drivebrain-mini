#include <FlowmeterComms.hpp>
#include <unistd.h>
#include <spdlog/spdlog.h>
#include <string>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstring>
#include <thread>
#include <poll.h>
#include <deque>

comms::FlowmeterComms::FlowmeterComms(const std::string &chip_path, uint32_t pin, core::ThreadSafeDeque<Pulse> &deque) 
:_deque(deque), _chip_path(chip_path), _pin(pin) { 
}

int comms::FlowmeterComms::open_gpio() {
    
    int chip_fd = open(_chip_path.c_str(), O_RDONLY);
    if (chip_fd < 0) {
        spdlog::error("Failed to connect to {}. Error: {}",_chip_path, strerror(errno));
        return -1;
    }

    struct gpio_v2_line_request request;
    memset(&request, 0, sizeof(request));
    request.offsets[0] = _pin;
    request.num_lines = 1;
    request.config.flags =  GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_RISING;

    strncpy(request.consumer, "flowmeterConsumer", sizeof(request.consumer) -1);
    int res = ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request);
    if (res < 0) {
        spdlog::error("Failed to config GPIO {}. Error {}",_pin, strerror(errno));
        close(chip_fd);
        return -1;
    }

    _line_fd = request.fd;

    close(chip_fd);
    return 0;
}

void comms::FlowmeterComms::read_pulses() {
    struct pollfd poll_struct = {};
    poll_struct.fd = _line_fd;
    poll_struct.events = POLLIN;

    while (_running) {
        int ret = poll(&poll_struct, 1, 100);
        if (ret < 0) {

            if (errno == EINTR) {
                continue;
            }
            spdlog::error("polling failed on GPIO {}. Error {}", _pin, strerror(errno));
            break;

        } else if (ret == 0) {
            continue;
        }

        struct gpio_v2_line_event event = {};
       
        ssize_t n = read(_line_fd, &event, sizeof(event));
        if (n != sizeof(event)) {
            spdlog::error("reading event failed on GPIO {}. Error {}", _pin, strerror(errno));
            break;
        }

        Pulse pulse;
        pulse.timestamp_ns = event.timestamp_ns;
        pulse.seqno = event.line_seqno;

        {
            std::lock_guard<std::mutex> lock(_deque.mtx);
            _deque.deque.push_back(pulse);
        }

        _deque.cv.notify_one();
    }
}

void comms::FlowmeterComms::start() {
    if (_line_fd < 0) {
        spdlog::error("Error, no GPIO found");
        return;
    }    
    _thread = std::thread(&comms::FlowmeterComms::read_pulses, this);
}


comms::FlowmeterComms::~FlowmeterComms() {
    
    _running = false;
    if (_thread.joinable()) {
        _thread.join();
    }

    if (_line_fd < 0) {
        return;
    } else {
        close(_line_fd);
    }

}

comms::FlowmeterHandler::FlowmeterHandler(core::ThreadSafeDeque<Pulse> &deque) :_deque(deque) {
}

void comms::FlowmeterHandler::start() {
    if (_thread.joinable()) {
        spdlog::warn("FlowmeterHandler already started");
        return;
    }
    _thread = std::thread(&comms::FlowmeterHandler::handle_pulses, this);
}

const comms::FlowMeterRead comms::FlowmeterHandler::get_latest() {
    std::lock_guard<std::mutex> lock(_latest_mutex);
    return _latest;
}

void comms::FlowmeterHandler::handle_pulses() {
    FlowMeterRead reading;
    std::deque<Pulse> local;

    while (true) {
        {
            std::unique_lock<std::mutex> lock(_deque.mtx);

            // Sleep until there's data or we're told to stop
            _deque.cv.wait(lock, [this] {
                return !_deque.deque.empty() || !_running;
            });

            // Stop only once the deque is drained, so no pulses are dropped on shutdown
            if (!_running && _deque.deque.empty()) {
                break;
            }

            local.swap(_deque.deque);
        } 

        for (const Pulse &pulse : local) {
            reading.pulse_count++;

            if (_last_seqno != 0 && pulse.seqno > _last_seqno + 1) {
                spdlog::warn("Missed {} flowmeter pulses", pulse.seqno - _last_seqno - 1);
            }

            _last_seqno = pulse.seqno;
        }
        local.clear();

        reading.total_volume_l = reading.pulse_count / PULSES_PER_LITER;

        {
            std::lock_guard<std::mutex> lock(_latest_mutex);
            _latest = reading;
        }
    }
}

comms::FlowmeterHandler::~FlowmeterHandler() {
    {
        std::lock_guard<std::mutex> lock(_deque.mtx);
        _running = false;
    }

    _deque.cv.notify_all();

    if (_thread.joinable()) {
        _thread.join();
    }
    spdlog::info("FlowmeterHandler stopped");
}