#include "helpers.h"

int main()
{
    SingleUSRP_B210 usrp(std::string(""));


    // By default, the antenna is always RX2 instead of TX/RX
    for (size_t i = 0; i < 2; ++i)
    {
        printf("\n\n====== Using channel %zd\n", i);
        for (std::string& ant : usrp()->get_rx_antennas(i))
            printf("Available antenna: %s %s\n", 
                   ant.c_str(),
                   usrp()->get_rx_antenna(i) == ant ? " (selected)" : "");

        auto rx_stream = usrp.make_single_rx_stream(
            10.0,
            1e9,
            50e3,
            240e3,
            i
        );

        // Attempt to start threaded streamer
        rx_stream->start();
        rx_stream->start(); // should show it already started
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        rx_stream->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // start again
        rx_stream->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        rx_stream->stop();
    }

    // Try TX streamers
    for (size_t i = 0; i < 2; ++i)
    {
        auto tx_stream = usrp.make_single_tx_stream(
            10.0,
            1e9,
            0,
            240e3,
            i
        );

        tx_stream->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        tx_stream->stop();
    }

    std::cout << "Complete" << std::endl;

    return 0;
}

