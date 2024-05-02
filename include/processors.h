#pragma once

/*
A collection of processor classes that are designed to work
with the streamer containers in helpers.h.
*/


#include <cstring>
#include <vector>
#include <iostream>

#include "helpers.h"
#include "timer.h"
#include "uhd/types/metadata.hpp"

class Processor
{
public:
    Processor() : m_lk(m_mtx) {}

protected:
    std::mutex m_mtx;
    std::unique_lock<std::mutex> m_lk;
};

/*
Sample class to copy a fixed number of iterations, and accumulate
properly into a longer buffer.

Ideally, you wouldn't want to just copy things out to a buffer alone,
but do some processing on the way as well, and store the results instead.

TODO: maybe refactor out the vector holding / copy method parts separately?
*/
template <typename T>
class CopyToBuffer : public Processor
{
public:
    CopyToBuffer(size_t size) : Processor(), m_buffer(size) {}

    // Handles the copying of data from the streamer to the buffer
    // This is a blocking call.
    void copy(std::shared_ptr<ThreadedRXStreamer> rx_stream, int max_iter=10)
    {
        // get the receive rate so we can use it in the tick count
        double rxrate = rx_stream->get_usrp()->get_rx_rate(
            rx_stream->get_stream_args().channels.at(0) // get the (assumed) only channel
        );
        std::cout << "CopyToBuffer: rxrate: " << rxrate << std::endl;

        size_t num_read_samples = 0;
        int target_bufIdx = -1;

        auto& cv = rx_stream->get_buffer_cv();
        for (int i = 0; i < max_iter; i++)
        {
            cv.wait(
                m_lk,
                [&]{
                    return (rx_stream->buffer_num_samps(0) > 0) || (rx_stream->buffer_num_samps(1) > 0) ;
                }
            );

            // Check which buffer is used
            if ((num_read_samples = rx_stream->buffer_num_samps(0)) > 0)
                target_bufIdx = 0;
            else if ((num_read_samples = rx_stream->buffer_num_samps(1)) > 0)
                target_bufIdx = 1;

            if (target_bufIdx >= 0 && target_bufIdx < 2)
            {
                HighResolutionTimer timer;

                // Lock the correct mutex
                std::lock_guard<std::mutex> lkg(
                    rx_stream->get_buffer_mutex(target_bufIdx)
                );
                std::cout << "buffer " << target_bufIdx << " has " << num_read_samples << " samples" << std::endl;

                // check the metadata time?
                auto& metadata = rx_stream->get_metadata(target_bufIdx);
                // if we use the rx_rate as the tick rate, we can see the number of samples
                std::cout << metadata.time_spec.to_ticks(rxrate) << std::endl;

                // we shift our internal buffer if necessary
                shift_back(num_read_samples);

                // then copy it in
                insert_samples(rx_stream->get_buffer(target_bufIdx).data(), 
                               num_read_samples,
                               metadata.time_spec.to_ticks(rxrate));

                // reset to 'consume'
                rx_stream->buffer_num_samps(target_bufIdx) = 0;
            }
        }

    }

    const std::vector<T>& get_buffer() const { return m_buffer; }
    void reset()
    {
        m_buffer_start_tick = 0;
        m_buffer.assign(m_buffer.size(), static_cast<T>(0)); 
    }


private:
    std::vector<T> m_buffer;
    size_t m_size_used = 0; // used to keep track of how much space has real data
    long long m_buffer_start_tick = 0;

    // Helper method to ensure that the incoming data will fit in the buffer
    void shift_back(size_t incoming_size)
    {
        // Check how much empty space we have
        if (m_buffer.size() - m_size_used < incoming_size)
        {
            const size_t offset_to_move = m_size_used + incoming_size - m_buffer.size();
            printf("Moving %zu elements to the front from %zd\n", m_size_used-offset_to_move, offset_to_move);
            std::memmove(
                m_buffer.data(), // move to the front
                &m_buffer.at(offset_to_move), // start from some ways away
                sizeof(T) * (m_size_used-offset_to_move) // copy everything currently used
            );

            // Define the new used value
            m_size_used -= offset_to_move;
            m_buffer_start_tick += offset_to_move;
        }
    }

    void insert_samples(const void* samples, size_t num_samples, long long ticks)
    {
        printf("CopyToBuffer: Copied %zd samples\n", num_samples);

        // update the start tick if it's the first one
        if (m_buffer_start_tick == 0)
            m_buffer_start_tick = ticks;

        std::memcpy(
            &m_buffer.at(m_size_used),
            samples,
            sizeof(T) * num_samples
        );
        m_size_used += num_samples;
    }

};


/*
This processor attempts to directly copy the receive stream data
into the transmit stream data. This should work in a timeline as follows:

RX Buffer 0
XXXXXXXXXXX-----------XXXXXXXXXXX
                |
           TX Buffer 0

RX Buffer 1
-----------XXXXXXXXXXX-----------
                           |
                      TX Buffer 1

TX Buffer 0
***********-----------***********
     |                     |
Dummy Data            1st RX Data

TX Buffer 1
-----------***********-----------***********
                |                     |
           Dummy Data            1st RX Data


As a processor class, this isn't particularly complicated, but it serves
as a good demonstration of the possibility of changing TX output quickly
based on RX input.

NOTE: this class implicitly expects the TX and RX streams to be synchronized in time.
Otherwise, the above timeline will not be obeyed, and the buffer indices used at a given
point in time will not necessarily be the same.
*/
template <typename T>
class Rebroadcast : public Processor
{
public:
    // Don't really need a ctor?

    void rebro(
        std::shared_ptr<ThreadedRXStreamer> rx_stream,
        std::shared_ptr<ThreadedTXStreamer> tx_stream,
        int max_iter=100
    ){
        size_t num_read_samples = 0;
        int target_bufIdx = -1;

        auto& cv = rx_stream->get_buffer_cv();
        for (int i = 0; i < max_iter; i++)
        {
            cv.wait(
                m_lk,
                [&]{
                    return (rx_stream->buffer_num_samps(0) > 0) || (rx_stream->buffer_num_samps(1) > 0) ;
                }
            );

            // Check which buffer is used
            if ((num_read_samples = rx_stream->buffer_num_samps(0)) > 0)
                target_bufIdx = 0;
            else if ((num_read_samples = rx_stream->buffer_num_samps(1)) > 0)
                target_bufIdx = 1;

            if (target_bufIdx >= 0 && target_bufIdx < 2)
            {
                HighResolutionTimer timer;

                // Lock the correct mutex
                std::lock_guard<std::mutex> lkg(
                    rx_stream->get_buffer_mutex(target_bufIdx)
                );
                // Similarly for the tx stream
                std::lock_guard<std::mutex> lkg2(
                    tx_stream->get_buffer_mutex(target_bufIdx)
                );
                std::cout << "buffer " << target_bufIdx << " has " << num_read_samples << " samples" << std::endl;

                // copy directly into the tx buffer
                if (
                        tx_stream->get_buffer(target_bufIdx).size() / sizeof(T) ==
                        num_read_samples
                ){
                    std::memcpy(
                        tx_stream->get_buffer(target_bufIdx).data(),
                        rx_stream->get_buffer(target_bufIdx).data(),
                        sizeof(T) * num_read_samples
                    );
                }
                else{
                    printf("Rebroadcast: Buffer sizes don't match! %zd vs %zd\n"
                           , tx_stream->get_buffer(target_bufIdx).size()
                           , num_read_samples
                        );
                }


                // reset to 'consume'
                rx_stream->buffer_num_samps(target_bufIdx) = 0;
            }
        }
    }
};
