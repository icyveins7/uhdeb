#include "helpers.h"

int main()
{
    SingleUSRP_B210 usrp("");

    // There's 2 channels here, test them separately
    // for (size_t i = 0; i < 2; ++i)
    // {
    //     usrp.configure_single_tx_channel(30.0, 1e9, 240e3, i);
    //     auto tx_stream = usrp.make_single_tx_stream(i);
    //     std::cout << "tx stream has " << tx_stream->get_num_channels() << " channels" << std::endl;
    // }


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
    //
    // std::cout << "Complete" << std::endl;

    return 0;
}

