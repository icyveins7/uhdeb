#include "helpers.h"

int main()
{
    SingleUSRP_B205mini usrp(std::string(""));

    // usrp.configure_single_tx_channel(30.0, 1e9, 240e3);
    // auto tx_stream = usrp.make_single_tx_stream();
    // std::cout << "tx stream has " << tx_stream->get_num_channels() << " channels" << std::endl;

    usrp.configure_single_rx_channel(10.0, 1e9, 240e3);
    auto rx_stream = usrp.make_single_rx_stream();

    // Attempt to start threaded streamer
    rx_stream->start();
    rx_stream->start(); // should show it already started

    auto& cv = rx_stream->get_buffer_cv();
    std::mutex useless_mutex;
    std::unique_lock<std::mutex> lk(useless_mutex); // this is a temporary (read: useless) lock constructed just to use the cv
    for (int i = 0; i < 100; i++)
    {
        cv.wait(
            lk,
            [&]{
                return (rx_stream->buffer_num_samps(0) > 0) || (rx_stream->buffer_num_samps(1) > 0) ;
            }
        );
        printf("Triggered %d\n", i);
        // Check which buffer is used
        if (rx_stream->buffer_num_samps(0) > 0)
        {
            std::lock_guard<std::mutex> lkg(
                rx_stream->get_buffer_mutex(0)
            );
            std::cout << "buffer 0 has " << rx_stream->buffer_num_samps(0) << " samples" << std::endl;

            // reset to simulate 'consumption'
            rx_stream->buffer_num_samps(0) = 0;
        }
        else if (rx_stream->buffer_num_samps(1) > 0)
        {
            std::lock_guard<std::mutex> lkg(
                rx_stream->get_buffer_mutex(1)
            );
            std::cout << "buffer 1 has " << rx_stream->buffer_num_samps(1) << " samples" << std::endl;

            // reset to simulate 'consumption'
            rx_stream->buffer_num_samps(1) = 0;
        }
    }
    rx_stream->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // start again
    rx_stream->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    rx_stream->stop();

    std::cout << "Complete" << std::endl;

    return 0;
}
