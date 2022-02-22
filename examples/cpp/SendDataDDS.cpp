#include "cerealHelper.hpp"
#include <Eigen/Dense>
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

using namespace org::eclipse::cyclonedds;

static std::vector<unsigned char> toVector(std::stringstream& ss)
{
    // discover size of data in stream
    ss.seekg(0, std::ios::beg);
    auto bof = ss.tellg();
    ss.seekg(0, std::ios::end);
    auto stream_size = std::size_t(ss.tellg() - bof);
    ss.seekg(0, std::ios::beg);

    // make your vector long enough
    std::vector<unsigned char> v(stream_size);

    // read directly in
    ss.read((char*)v.data(), std::streamsize(v.size()));

    return v;
}

int main(int argc, char** argv)
{
    // --- Processing Normal (Numpy) Matrix

    Eigen::Matrix3d rotMat;
    Eigen::Affine3d htMat{Eigen::Affine3d::Identity()};
    rotMat = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX());
    htMat.translate(Eigen::Vector3d(20, 30, 40));
    htMat.rotate(rotMat);

    std::stringstream eigenStream;
    cereal::BinaryOutputArchive outputArchive(eigenStream);
    outputArchive(CEREAL_NVP(htMat));
    ExampleIdlData::BytesArray payloadEigen = toVector(eigenStream);

    std::cout << "htMat: " << std::endl << htMat.matrix() << std::endl;

    // ---

    // --- Send through DDS

    /* First, a domain participant is needed.
     * Create one on the default domain. */
    dds::domain::DomainParticipant participant(domain::default_id());

    /* To publish something, a topic is needed. */
    dds::topic::Topic<ExampleIdlData::Msg> topic(participant, "example_topic");

    /* A writer also needs a publisher. */
    dds::pub::Publisher publisher(participant);

    /* Now, the writer can be created to publish a HelloWorld message. */
    dds::pub::DataWriter<ExampleIdlData::Msg> writer(publisher, topic);

    /* For this example, we'd like to have a subscriber to actually read
     * our message. This is not always necessary. Also, the way it is
     * done here is just to illustrate the easiest way to do so. It isn't
     * really recommended to do a wait in a polling loop, however.
     * Please take a look at Listeners and WaitSets for much better
     * solutions, albeit somewhat more elaborate ones. */
    std::cout << "=== [Publisher] Waiting for subscriber." << std::endl;
    while (writer.publication_matched_status().current_count() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    /* Contruct the message including payloads */
    ExampleIdlData::Msg msg;
    msg.id(32);                    // just an example
    msg.message("hello there!!!"); // just an example
    msg.payloadEigen(payloadEigen);

    /* Write the message. */
    std::cout << "=== [Publisher] Write sample." << std::endl;
    writer.write(msg);

    /* With a normal configuration (see dds::pub::qos::DataWriterQos
     * for various different writer configurations), deleting a writer will
     * dispose all its related message.
     * Wait for the subscriber to have stopped to be sure it received the
     * message. Again, not normally necessary and not recommended to do
     * this in a polling loop. */
    std::cout << "=== [Publisher] Waiting for sample to be accepted." << std::endl;
    while (writer.publication_matched_status().current_count() > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // ---

    return 0;
}