#include "npyHelper.hpp"
#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>

/* Include the C++ DDS API. */
#include "dds/dds.hpp"

/* Include data type and specific traits to be used with the C++ DDS API. */
#include "ExampleIdlData.hpp"

namespace po = boost::program_options;
using namespace org::eclipse::cyclonedds;

static inline std::stringstream toString(const std::vector<unsigned char>& data) {
    std::stringstream ss;
    ss.write((char const*)data.data(), std::streamsize(data.size()));
    return ss;
}

void processMsgPython(const ExampleIdlData::Msg& msg);

int main(int argc, char** argv)
{
    // --- Send through DDS

    /* First, a domain participant is needed.
     * Create one on the default domain. */
    dds::domain::DomainParticipant participant(domain::default_id());

    /* To publish something, a topic is needed. */
    dds::topic::Topic<ExampleIdlData::Msg> topic(participant, "example_topic");

    /* A reader also needs a subscriber. */
    dds::sub::Subscriber subscriber(participant);

    /* Now, the reader can be created to subscribe to a HelloWorld message. */
    dds::sub::DataReader<ExampleIdlData::Msg> reader(subscriber, topic);

    /* Poll until a message has been read.
     * It isn't really recommended to do this kind wait in a polling loop.
     * It's done here just to illustrate the easiest way to get data.
     * Please take a look at Listeners and WaitSets for much better
     * solutions, albeit somewhat more elaborate ones. */
    std::cout << "=== [Subscriber] Wait for message." << std::endl;
    bool poll = true;
    while (poll) {
        /* For this example, the reader will return a set of messages (aka
         * Samples). There are other ways of getting samples from reader.
         * See the various read() and take() functions that are present. */
        dds::sub::LoanedSamples<ExampleIdlData::Msg> samples;

        /* Try taking samples from the reader. */
        samples = reader.take();

        /* Are samples read? */
        if (samples.length() > 0) {
            /* Use an iterator to run over the set of samples. */
            dds::sub::LoanedSamples<ExampleIdlData::Msg>::const_iterator sample_iter;
            for (sample_iter = samples.begin(); sample_iter < samples.end(); ++sample_iter) {
                /* Get the message and sample information. */
                const ExampleIdlData::Msg& msg = sample_iter->data();
                const dds::sub::SampleInfo& info = sample_iter->info();

                /* Sometimes a sample is read, only to indicate a data
                 * state change (which can be found in the info). If
                 * that's the case, only the key value of the sample
                 * is set. The other data parts are not.
                 * Check if this sample has valid data. */
                if (info.valid()) {
                    std::cout << "=== [Subscriber] Message received:" << std::endl;
                    std::cout << "    userID  : " << msg.id() << std::endl;
                    std::cout << "    Message : \"" << msg.message() << "\"" << std::endl;

                    processMsgPython(msg);

                    /* Only 1 message is expected in this example. */
                    poll = false;
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    // ---
    return 0;
}


void processMsgPython(const ExampleIdlData::Msg& msg) {
    
    // --- Processing Normal (Numpy) Matrix

    auto eigenStream = toString(msg.payloadEigen());
    std::vector<unsigned long> matrixShape;
    std::vector<double> matData;
    bool fortran_order = false;
    npy::LoadArrayFromString(eigenStream, matrixShape, fortran_order, matData);
    Eigen::MatrixXd eigenMat =
        Eigen::Map<Eigen::MatrixXd>(matData.data(), matrixShape[1], matrixShape[0]);
    eigenMat.transposeInPlace();
    std::cout << "htMat: " << std::endl << eigenMat << std::endl;

    // ---

}