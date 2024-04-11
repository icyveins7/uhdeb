#include <vector>
#include <stdint.h>
#include <fstream>

struct sc16sample
{
    int16_t real = 0;
    int16_t imag = 0;

    std::ostream& operator<<(std::ostream& stream) const
    {
        stream << real << imag;
        return stream;
    }
};

constexpr sc16sample qpsk[4] = {
    {1, 0},
    {0, 1},
    {-1, 0},
    {0, -1}
};

int main()
{
    int numPreambleSamples = 2000;
    int numDataSamples = 8000;
    int numBurstSamples = numPreambleSamples + numDataSamples;

    int numGuardSamples = 10000;
    int numPeriodSamples = numBurstSamples + numGuardSamples;

    int numBursts = 2;
    int16_t A = 10000;

    // Generate preamble
    std::vector<sc16sample> preamble(numPreambleSamples);
    printf("Generating preamble of length %d\n", numPreambleSamples);
    for (int i = 0; i < numPreambleSamples; ++i)
    {
        preamble.at(i) = qpsk[rand() % 4];
    }

    // Generate signal
    std::vector<sc16sample> signal(numPeriodSamples * numBursts);
    for (int j = 0; j < numBursts; j++)
    {
        printf("Generating burst %d\n", j);
        // Copy preamble
        for (int i = 0; i < numPreambleSamples; ++i)
        {
            signal.at(i + j * numPeriodSamples).real = preamble.at(i).real * A;
            signal.at(i + j * numPeriodSamples).imag = preamble.at(i).imag * A;
        }

        // Generate data
        for (int i = 0; i < numDataSamples; ++i)
        {
            sc16sample sample = qpsk[rand() % 4];
            signal.at(i + j * numPeriodSamples + numPreambleSamples).real = sample.real * A;
            signal.at(i + j * numPeriodSamples + numPreambleSamples).imag = sample.imag * A;
        }
    }

    // Write to file
    std::ofstream outFile("generated_bursty_qpsk.bin", std::ios::out | std::ios::binary);
    outFile.write(reinterpret_cast<const char*>(signal.data()), signal.size() * sizeof(sc16sample));
    outFile.close();


    return 0;
}
