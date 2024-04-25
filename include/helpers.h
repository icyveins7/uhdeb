#pragma once

#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/thread.hpp>

#include <iostream>
// #include <boost/algorithm/string.hpp>
// #include <boost/filesystem.hpp>
#include <boost/format.hpp>
// #include <boost/program_options.hpp>
// #include <cmath>
// #include <csignal>
// #include <fstream>
// #include <iterator>
// #include <algorithm>
// #include <functional>
// #include <iostream>
// #include <thread>
// #include <condition_variable>


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
        const uhd::device_addr_t& dev_addr, 
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
    uhd::tx_streamer::sptr make_single_tx_stream(
        size_t chnl_no = 0,
        std::string cpu_fmt = "sc16",
        std::string otw_fmt = "sc16"
    ){
        uhd::stream_args_t stream_args(cpu_fmt, otw_fmt);
        stream_args.channels = {chnl_no};
        return usrp->get_tx_stream(stream_args);
    }

    // Helper to configure a single TX channel
    void configure_single_tx_channel(
        double gain,
        double freq_Hz,
        double rate,
        size_t chnl_no = 0
    ){
        usrp->set_tx_freq(
            uhd::tune_request_t(freq_Hz),
            chnl_no
        );
        std::cout << boost::format("Actual TX Freq: %f MHz...")
                         % (usrp->get_tx_freq(chnl_no) / 1e6)
                  << std::endl
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
    uhd::rx_streamer::sptr make_single_rx_stream(
        size_t chnl_no = 0,
        std::string cpu_fmt = "sc16",
        std::string otw_fmt = "sc16"
    ){
        uhd::stream_args_t stream_args(cpu_fmt, otw_fmt);
        stream_args.channels = {chnl_no};
        return usrp->get_rx_stream(stream_args);
    }

    void configure_single_rx_channel(
        double gain,
        double freq_Hz,
        double rate,
        size_t chnl_no = 0
    ){
        usrp->set_rx_freq(
            uhd::tune_request_t(freq_Hz),
            chnl_no
        );
        std::cout << boost::format("Actual RX Freq: %f MHz...")
                         % (usrp->get_rx_freq(chnl_no) / 1e6)
                  << std::endl
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
        const uhd::device_addr_t& dev_addr,
        const std::string clock_src = "internal",
        const std::string time_src = "internal",
        bool verbose = true
    ) : SingleUSRP(dev_addr, "A:A", "A:A", clock_src, time_src, verbose)
    {

    }
};


