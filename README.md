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

## rx_samples_to_file_buffered
This aims to be an easily configurable receiver executable that is modified from the ```rx_samples_to_file``` example.

You should run it with ```rx_samples_to_file_buffered --h``` to get good documentation on how to configure it.

Changes include:
1. Ability to set both clock and time sources to gpsdo i.e. ```--ref gpsdo```.
2. Help message documentation for different daughterboards, including B210 and TwinRX.
3. Threaded saves of data to increase throughput in the ```recv()``` hot loop.
4. Time-tagged (per second) data files, saved into channel index-labelled subfolders.
5. Automatic creation of data folder.
6. Ability to automatically resync and then restart recording when the recording breaks due to errors with ```--continue```.
7. Ability to selectively save data only when at least 1 sample exceeds a certain threshold with ```--threshold```. This facilitates small disk footprints when signal is sparse in time.
8. Ability to check for saturation with ```--saturation_warning``` in case the signal power is near discretization limits.


