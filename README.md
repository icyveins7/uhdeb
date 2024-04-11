# uhdeb
This is a bunch of UHD executables/binaries, which offer some extensions and/or some changes to the default UHD examples.

# Building

Build using CMake.

## Known Issues

1. If you built UHD in Release Mode (which is the recommendation from the docs), you must build these executables in Release mode as well, otherwise there may be errors during runtime (not compile time!).

# List of Extensions
## tx_file_while_rx
This aims to transmit a file of sc16 samples repeatedly, while receiving.
It is modified from the ```txrx_loopback_to_file.cpp``` example.

Changes include:
1. Only 1 TX streamer is enforced.
2. The file is entirely preloaded before being used, to prevent delays in the ```send``` loop.
3. The file is assumed to be of type sc16, and this is enforced in the ```stream_args``` for TX.
4. RX is performed via a separate thread for writing to disk.



