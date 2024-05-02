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


/*
Sample class to copy a fixed number of iterations, and accumulate
properly into a longer buffer.

Ideally, you wouldn't want to just copy things out to a buffer alone,
but do some processing on the way as well, and store the results instead.

TODO: maybe refactor out the vector holding / copy method parts separately?
*/
template <typename T>
class CopyToBuffer
{
public:
    CopyToBuffer(size_t size) : m_buffer(size), m_lk(m_mtx) {}

    // Handles the copying of data from the streamer to the buffer
    void copy(std::shared_ptr<ThreadedRXStreamer> rx_stream, int max_iter=10)
    {
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
            printf("Triggered %d\n", i);

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

                // we shift our internal buffer if necessary
                shift_back(num_read_samples);

                // then copy it in
                insert_samples(rx_stream->get_buffer(target_bufIdx).data(), num_read_samples);

                // reset to 'consume'
                rx_stream->buffer_num_samps(target_bufIdx) = 0;
            }
        }

    }

    const std::vector<T>& get_buffer() const { return m_buffer; }
    void reset(){ m_buffer.assign(m_buffer.size(), static_cast<T>(0)); }


private:
    std::mutex m_mtx;
    std::unique_lock<std::mutex> m_lk;
    std::vector<T> m_buffer;
    size_t m_size_used = 0; // used to keep track of how much space has real data

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
        }
    }

    void insert_samples(const void* samples, size_t num_samples)
    {
        printf("CopyToBuffer: Copied %zd samples\n", num_samples);
        std::memcpy(
            &m_buffer.at(m_size_used),
            samples,
            sizeof(T) * num_samples
        );
        m_size_used += num_samples;
    }

};
