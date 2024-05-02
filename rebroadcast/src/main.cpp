#include "processors.h"
#include "helpers.h"

int main()
{
    SingleUSRP usrp;

    auto txstream = usrp.make_single_tx_stream(
        20, 1e9, 0, 240e3
    );
    auto rxstream = usrp.make_single_rx_stream(
        20, 1e9, 0, 240e3
    );

    Rebroadcast<std::complex<short>> processor;

    txstream->start();
    rxstream->start();
    processor.rebro(
        rxstream,
        txstream
    );

    txstream->stop();
    rxstream->stop();

    return 0;
}
