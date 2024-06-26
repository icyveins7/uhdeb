#pragma once

#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/thread.hpp>

#include <iostream>
#include <boost/format.hpp>
#include <iostream>
#include <thread>
#include <condition_variable>
#include <vector>

/*
Encapsulation for streamers.

This helps you manage the buffers and threads for simple cases.

The implementation allows for the following:
- Internal thread (started on ctor) which runs the streamer (recv/send loop)
- Control of the thread (start/stop) via condition variable signalling
- send/recv is done via on-demand stream_cmds/metadata (no time spec)
- Internal cleanup of thread and buffers upon destruction
- Dual buffer swapping implementation; either read or write from/to buffer 0, 1, 0, ...
*/

enum ThreadedStreamerCmd : uint8_t
{
    STOP,
    START,
    RUNNING,
    EXIT
};

// CRTP woohoo
// T: CRTP derived class
// U: derived class streamer sptr
template <typename T, typename U>
class ThreadedStreamer
{
public:
    ThreadedStreamer(
        uhd::usrp::multi_usrp::sptr usrp,
        std::vector<size_t>& chnl_nos,
        std::string cpu_fmt = "sc16",
        std::string otw_fmt = "sc16"
    ) : m_chnl_nos{chnl_nos}, m_stream_args(cpu_fmt, otw_fmt), m_usrp(usrp)
    {
        impl()->create_stream();

        allocate_buffers();

        // Begin the thread, but not the internal loop
        m_state = ThreadedStreamerCmd::STOP;
        m_thread = std::thread(
            &T::thread_work,
            impl()
        );
    }

    ~ThreadedStreamer()
    {
        // End the thread gracefully
        m_state = ThreadedStreamerCmd::EXIT;
        m_cv.notify_one(); // we notify in case it is currently waiting

        m_thread.join();
    }

    void start()
    {
        if (m_state == ThreadedStreamerCmd::STOP)
        {
            m_buffs_num_samps[0] = 0;
            m_buffs_num_samps[1] = 0;

            m_state = ThreadedStreamerCmd::START;
            m_cv.notify_one();
        }
        else
        {
            printf("Streamer is already running\n");
        }
    }

    // This is non-blocking i.e. this function will return immediately
    // but the internal thread will finish its current loop iteration before stopping
    void stop()
    {
        printf("Stopping streamer...\n");
        m_state = ThreadedStreamerCmd::STOP;
        // don't need a notify here since it's in the receive inner loop
    }

    std::mutex& get_buffer_mutex(int i){ return m_buffs_mtx[i]; }
    std::condition_variable& get_buffer_cv(){ return m_buffs_cv; }
    size_t& buffer_num_samps(int i){ return m_buffs_num_samps[i]; }
    std::vector<char>& get_buffer(int i){ return m_buffs[i]; }
    uhd::usrp::multi_usrp::sptr get_usrp() { return m_usrp; }
    uhd::stream_args_t& get_stream_args() { return m_stream_args; }
    volatile uint8_t& get_state() { return m_state; }

protected:
    std::vector<size_t> m_chnl_nos;
    uhd::stream_args_t m_stream_args;
    U m_stream;
    uhd::usrp::multi_usrp::sptr m_usrp;

    size_t m_max_num_samps;
    std::vector<char> m_buffs[2];
    int m_bufIdx = 0;
    std::mutex m_buffs_mtx[2];
    std::condition_variable m_buffs_cv;
    size_t m_buffs_num_samps[2];

    std::thread m_thread;
    std::mutex m_mtx; // for control of the thread
    std::condition_variable m_cv; // for control of the thread
    volatile uint8_t m_state = 0;

    // CRTP implementation extractor helper
    T* impl()
    {
        return static_cast<T*>(this);
    }


    // This is the method that creates an RX/TX stream in derived classes
    void create_stream()
    {
    }

    // Helps to allocate the buffers
    void allocate_buffers()
    {
        // // use dual buffer system
        // m_buffs.resize(2);

        if (m_stream_args.cpu_format == "sc16")
        {
            // 4 bytes per sample
            for (int i = 0; i < 2; ++i)
            {
                m_buffs[i].resize(m_max_num_samps * 4);
            }
        }
        else if (m_stream_args.cpu_format == "fc32")
        {
            // 8 bytes per sample
            for (int i = 0; i < 2; ++i)
            {
                m_buffs[i].resize(m_max_num_samps * 8);
            }
        }
        else
        {
            throw std::runtime_error("Only sc16 or fc32 supported for now");
        }
    }

    void thread_work()
    {

    }
};

// TODO: this class currently only works for a single channel.
class ThreadedRXStreamer : public ThreadedStreamer<ThreadedRXStreamer, uhd::rx_streamer::sptr>
{
    friend class ThreadedStreamer<ThreadedRXStreamer, uhd::rx_streamer::sptr>;

public:
    ThreadedRXStreamer(
        uhd::usrp::multi_usrp::sptr usrp,
        std::vector<size_t>& chnl_nos,
        std::string cpu_fmt = "sc16",
        std::string otw_fmt = "sc16"
    ) : ThreadedStreamer<ThreadedRXStreamer, uhd::rx_streamer::sptr>(
        usrp,
        chnl_nos,
        cpu_fmt,
        otw_fmt
    )
    {
    }

    uhd::rx_metadata_t& get_metadata(int i){ return m_md[i]; }


private:
    uhd::rx_metadata_t m_md[2];

    void create_stream()
    {
        m_stream_args.channels = m_chnl_nos;
        m_stream = m_usrp->get_rx_stream(m_stream_args);
        m_max_num_samps = m_stream->get_max_num_samps();

        printf("Max num samps = %zd\n", m_max_num_samps);
    }

    void thread_work()
    {
        const double settling_time = 0.2f;
        double timeout;
        bool overflow_message = true;

        // Create vector of pointers to the buffers
        std::vector<void*> buff_ptrs(1);

        // Create lock for signalling
        std::unique_lock<std::mutex> lk(m_mtx);
        while (true)
        {
            m_cv.wait(lk, [&]{return m_state > ThreadedStreamerCmd::STOP;});

            // Begin receive loop
            if (m_state == ThreadedStreamerCmd::START)
            {
                printf("Issuing a new stream command to receive..\n");
                uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
                stream_cmd.stream_now = true;
                // stream_cmd.time_spec = m_usrp->get_time_now() + uhd::time_spec_t(0.5); // 0.5s settling time
                m_stream->issue_stream_cmd(stream_cmd);

                // Explicitly change the state
                m_state = ThreadedStreamerCmd::RUNNING;
                // Reset the timeout
                timeout = settling_time + 0.5f;

                // Reset the buffers
                m_bufIdx = 0;
                m_buffs_num_samps[0] = 0;
                m_buffs_num_samps[1] = 0;
            }
            else if (m_state == ThreadedStreamerCmd::EXIT)
            {
                break;
            }
            else
            {
                continue;
            }

            // Primary recv loop
            while (m_state == ThreadedStreamerCmd::RUNNING)
            {
                // Scope for the lock_guard
                {
                    // Lock the buffer index that is being used
                    std::lock_guard<std::mutex> rlock(m_buffs_mtx[m_bufIdx]);

                    // set the buffer index to recv into
                    buff_ptrs.at(0) = m_buffs[m_bufIdx].data();

                    // write directly into the variable that is signalled out
                    m_buffs_num_samps[m_bufIdx] = m_stream->recv(buff_ptrs, m_max_num_samps, m_md[m_bufIdx], timeout);
                    timeout             = 0.1f; // small timeout for subsequent recv

                    // if (md.has_time_spec)
                    // {
                    //     std::cout << md.time_spec.to_ticks() << std::endl;
                    // }
                    // {
                    //     printf("====== RX -> Buffer [%d]: %zd samps\n", m_bufIdx, m_buffs_num_samps[m_bufIdx]);
                    // }

                    if (m_md[m_bufIdx].error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                        std::cout << "Timeout while streaming" << std::endl;
                        break;
                    }
                    if (m_md[m_bufIdx].error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
                        if (overflow_message) {
                            overflow_message = false;
                            std::cerr
                                << boost::format(
                                       "Got an overflow indication\n"); //. Please consider the following:\n"
                                       // "  Your write medium must sustain a rate of %fMB/s.\n"
                                       // "  Dropped samples will not be written to the file.\n"
                                       // "  Please modify this example for your purposes.\n"
                                       // "  This message will not appear again.\n")
                                       // % (usrp->get_rx_rate() * sizeof(samp_type) / 1e6);
                        }
                        continue;
                    }
                    if (m_md[m_bufIdx].error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                        throw std::runtime_error("Receiver error " + m_md[m_bufIdx].strerror());
                    }

                    // Change buffer
                    m_bufIdx = (m_bufIdx + 1) % 2;

                }
                // Signal that buffer has been filled before receiving into next buffer
                m_buffs_cv.notify_one();

            } // End of receive loop
            // End the stream
            uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
            m_stream->issue_stream_cmd(stream_cmd);
            printf("Receive loop has been stopped. Waiting for next command...\n");

        }
        printf("Exiting receive thread\n");

    }

};

// TODO: this class currently only works for a single channel.
class ThreadedTXStreamer : public ThreadedStreamer<ThreadedTXStreamer, uhd::tx_streamer::sptr>
{
    friend class ThreadedStreamer<ThreadedTXStreamer, uhd::tx_streamer::sptr>;

public:
    ThreadedTXStreamer(
        uhd::usrp::multi_usrp::sptr usrp,
        std::vector<size_t>& chnl_nos,
        std::string cpu_fmt = "sc16",
        std::string otw_fmt = "sc16"
    ) : ThreadedStreamer<ThreadedTXStreamer, uhd::tx_streamer::sptr>(
        usrp,
        chnl_nos,
        cpu_fmt,
        otw_fmt
    )
    {
    }

    uhd::tx_metadata_t& get_metadata(int i){ return m_md[i]; }


private:
    uhd::tx_metadata_t m_md[2];

    void create_stream()
    {
        m_stream_args.channels = m_chnl_nos;
        m_stream = m_usrp->get_tx_stream(m_stream_args);
        m_max_num_samps = m_stream->get_max_num_samps();

        printf("Max num samps = %zd\n", m_max_num_samps);
    }

    void thread_work()
    {
        const double settling_time = 0.2f;
        double timeout;
        bool overflow_message = true;

        // Create vector of pointers to the buffers
        std::vector<void*> buff_ptrs(1);

        // Create lock for signalling
        std::unique_lock<std::mutex> lk(m_mtx);
        while (true)
        {
            m_cv.wait(lk, [&]{return m_state > ThreadedStreamerCmd::STOP;});

            // Begin receive loop
            if (m_state == ThreadedStreamerCmd::START)
            {
                // For tx, just set the metadata values
                m_md[0].start_of_burst = true;
                m_md[0].has_time_spec = false;

                // Explicitly change the state
                m_state = ThreadedStreamerCmd::RUNNING;
                // Reset the timeout
                timeout = settling_time + 0.5f;

                // Reset the buffers
                m_bufIdx = 0;
                m_buffs_num_samps[0] = 0;
                m_buffs_num_samps[1] = 0;
            }
            else if (m_state == ThreadedStreamerCmd::EXIT)
            {
                break;
            }
            else
            {
                continue;
            }

            // Primary transmit loop
            while (m_state == ThreadedStreamerCmd::RUNNING)
            {
                // Scope for the lock_guard
                {
                    // Lock the buffer index that is being used
                    std::lock_guard<std::mutex> rlock(m_buffs_mtx[m_bufIdx]);

                    // set the buffer index to tx from
                    buff_ptrs.at(0) = m_buffs[m_bufIdx].data();

                    // write directly into the variable that is signalled out
                    m_buffs_num_samps[m_bufIdx] = m_stream->send(
                        buff_ptrs, m_max_num_samps, m_md[m_bufIdx]);
                    timeout             = 0.1f; // small timeout for subsequent recv
                    m_md[m_bufIdx].start_of_burst = false;

                    if (m_buffs_num_samps[m_bufIdx] != m_max_num_samps)
                        printf("Sent %zd of %zd samples\n",
                               m_buffs_num_samps[m_bufIdx],
                               m_max_num_samps
                               );

                    // Change buffer
                    m_bufIdx = (m_bufIdx + 1) % 2;

                }
                // Signal that buffer has been transmitted before receiving into next buffer
                m_buffs_cv.notify_one();

            } // End of receive loop
            // End the stream
            m_md[m_bufIdx].end_of_burst = true;
            m_stream->send("", 0, m_md[m_bufIdx]);
            printf("Transmit loop has been stopped. Waiting for next command...\n");

        }
        printf("Exiting transmit thread\n");

    }

};


/*
Outline a few container classes that help to
enable all the boards on specific USRP models.

This helps to enforce a natural ordered list of channels,
(i.e. left to right),
associated to the physical ports on the USRP.

Then the user/developer can select from the ordered
list of channels in order to create streams.
*/

class SingleUSRP
{
public:
    SingleUSRP(
        const uhd::device_addr_t dev_addr=uhd::device_addr_t(),
        const std::string full_tx_subdev_spec = "",
        const std::string full_rx_subdev_spec = "",
        const std::string clock_src = "internal",
        const std::string time_src = "internal",
        bool verbose = true
    ) : m_verbose(verbose)
    {
        // boilerplate
        usrp = uhd::usrp::multi_usrp::make(dev_addr);
        usrp->set_tx_subdev_spec(uhd::usrp::subdev_spec_t(full_tx_subdev_spec));
        usrp->set_rx_subdev_spec(uhd::usrp::subdev_spec_t(full_rx_subdev_spec));
        usrp->set_clock_source(clock_src);
        usrp->set_time_source(time_src);

        // print what we can?
        if (verbose)
        {
            std::cout << "Using TX subdev spec " << std::endl;
            std::cout << usrp->get_tx_subdev_spec().to_pp_string() << std::endl;
            std::cout << "Using RX subdev spec " << std::endl;
            std::cout << usrp->get_rx_subdev_spec().to_pp_string() << std::endl;

            std::cout << "Total " << usrp->get_tx_num_channels() << " TX channels" << std::endl;
            std::cout << "Total " << usrp->get_rx_num_channels() << " RX channels" << std::endl;

            std::cout << "Clock source: " << usrp->get_clock_source(0) << std::endl;
            std::cout << "Time source: " << usrp->get_time_source(0) << std::endl;

            std::cout << usrp->get_pp_string() << std::endl;
        }
    }

    void set_verbosity(bool v)
    {
        m_verbose = v;
    }

    // Helper to dereference the sptr object
    uhd::usrp::multi_usrp::sptr operator()() const
    {
        return usrp;
    }

    // Helper to get and configure a single TX stream for a single channel in 1 step
    // with some common defaults
    std::shared_ptr<ThreadedTXStreamer> make_single_tx_stream(
        double gain,
        double freq_Hz,
        double lo_offset_Hz,
        double rate,
        size_t chnl_no = 0,
        std::string cpu_fmt = "sc16",
        std::string otw_fmt = "sc16"
    ){
        configure_single_tx_channel(
            gain,
            freq_Hz,
            lo_offset_Hz,
            rate,
            chnl_no
        );

        return std::make_shared<ThreadedTXStreamer>(
            usrp,
            std::vector<size_t>{chnl_no},
            cpu_fmt, otw_fmt
        );
    }

    // Helper to configure a single TX channel
    void configure_single_tx_channel(
        double gain,
        double freq_Hz,
        double lo_offset_Hz,
        double rate,
        size_t chnl_no = 0
    ){
        auto tune_result = usrp->set_tx_freq(
            uhd::tune_request_t(freq_Hz, lo_offset_Hz),
            chnl_no
        );
        std::cout << boost::format("Actual TX RF Freq: %f MHz...")
                         % (tune_result.actual_rf_freq / 1e6)
                  << std::endl
                  << boost::format("Actual TX DSP Freq: %f MHz...")
                         % (tune_result.actual_dsp_freq / 1e6)
                  << std::endl;

        usrp->set_tx_gain(gain, chnl_no);
        std::cout << boost::format("Actual TX Gain: %f dB...")
            % usrp->get_tx_gain(chnl_no)
            << std::endl
            << std::endl;

        usrp->set_tx_rate(rate, chnl_no);
        std::cout << boost::format("Actual TX Rate: %f Msps...")
            % (usrp->get_tx_rate() / 1e6)
            << std::endl
            << std::endl;

    }

    // Helper to get and configure a single RX stream for a single channel in 1 step
    // with some common defaults
    std::shared_ptr<ThreadedRXStreamer> make_single_rx_stream(
        double gain,
        double freq_Hz,
        double lo_offset_Hz,
        double rate,
        size_t chnl_no = 0,
        std::string cpu_fmt = "sc16",
        std::string otw_fmt = "sc16"
    ){
        configure_single_rx_channel(
            gain,
            freq_Hz,
            lo_offset_Hz,
            rate,
            chnl_no
        );

        return std::make_shared<ThreadedRXStreamer>(
            usrp,
            std::vector<size_t>{chnl_no},
            cpu_fmt, otw_fmt
        );
    }

    void configure_single_rx_channel(
        double gain,
        double freq_Hz,
        double lo_offset_Hz,
        double rate,
        size_t chnl_no = 0
    ){
        auto tune_result_t = usrp->set_rx_freq(
            uhd::tune_request_t(freq_Hz, lo_offset_Hz),
            chnl_no
        );
        std::cout << boost::format("Actual RX RF Freq: %f MHz...")
                         % (tune_result_t.actual_rf_freq / 1e6)
                  << std::endl
                  << boost::format("Actual RX DSP Freq: %f MHz...")
                         % (tune_result_t.actual_dsp_freq / 1e6)
                  << std::endl;

        usrp->set_rx_gain(gain, chnl_no);
        std::cout << boost::format("Actual RX Gain: %f dB...")
            % usrp->get_rx_gain(chnl_no)
            << std::endl
            << std::endl;


        usrp->set_rx_rate(rate, chnl_no);
        std::cout << boost::format("Actual RX Rate: %f Msps...")
            % (usrp->get_rx_rate() / 1e6)
            << std::endl
            << std::endl;
    }

private:
    // Member variables
    uhd::usrp::multi_usrp::sptr usrp;
    bool m_verbose = true;
};

class SingleUSRP_B205mini : public SingleUSRP
{
public:
    SingleUSRP_B205mini(
        const uhd::device_addr_t dev_addr=uhd::device_addr_t(),
        const std::string clock_src = "internal",
        const std::string time_src = "internal",
        bool verbose = true
    ) : SingleUSRP(dev_addr, "A:A", "A:A", clock_src, time_src, verbose)
    {

    }
};


class SingleUSRP_B210 : public SingleUSRP
{
public:
    SingleUSRP_B210(
        const uhd::device_addr_t dev_addr=uhd::device_addr_t(),
        const std::string clock_src = "internal",
        const std::string time_src = "internal",
        bool verbose = true
    ) : SingleUSRP(dev_addr, "A:A A:B", "A:A A:B", clock_src, time_src, verbose)
    {

    }
};



