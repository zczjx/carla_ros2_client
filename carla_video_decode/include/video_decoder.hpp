#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>
#include <queue>
#include <cstdio>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <stdlib.h>
#include <string.h>
}

#include "sensor_msgs/msg/image.hpp"

namespace video_dec_node
{

class VideoEncoder
{
public:
    VideoEncoder(std::string &codec_name);

    std::shared_ptr<AVFrame> fillinFrame(std::shared_ptr<sensor_msgs::msg::Image> image_msg, int pts_idx);

    void flushEncode(std::shared_ptr<FILE> outfile);

    void encode(std::shared_ptr<AVFrame> frame, std::queue<std::shared_ptr<sensor_msgs::msg::Image>> &out_buffer);

    void encode(std::shared_ptr<AVFrame> frame, std::shared_ptr<FILE> outfile);


    ~VideoEncoder();

private:
    std::shared_ptr<const AVCodec> m_codec;
    std::string m_codec_name;
    std::shared_ptr<AVCodecContext> m_ctx{nullptr};
    std::shared_ptr<AVFrame> m_frame;
    std::shared_ptr<AVPacket> m_pkt;
};

}