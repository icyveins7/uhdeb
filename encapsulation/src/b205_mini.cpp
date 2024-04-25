#include "helpers.h"

int main()
{
    SingleUSRP_B205mini usrp("");

    usrp.configure_single_tx_channel(30.0, 1e9, 240e3);
    auto tx_stream = usrp.make_single_tx_stream();
    std::cout << "tx stream has " << tx_stream->get_num_channels() << " channels" << std::endl;

    usrp.configure_single_rx_channel(10.0, 1e9, 240e3);
    auto rx_stream = usrp.make_single_rx_stream();

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

    std::cout << "Complete" << std::endl;

    return 0;
}

