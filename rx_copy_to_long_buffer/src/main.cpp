#include "helpers.h"
#include "processors.h"

int main()
{
    SingleUSRP usrp;
    CopyToBuffer<std::complex<short>> processor(10000);

    auto rx_stream = usrp.make_single_rx_stream(30, 1e9, 0, 2.4e6);

    rx_stream->start();

    processor.copy(rx_stream);

    rx_stream->stop();

    return 0;
}
