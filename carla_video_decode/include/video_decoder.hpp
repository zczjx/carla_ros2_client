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
#include <libavutil/pixfmt.h>
#include <libavutil/imgutils.h>
#include <stdlib.h>
#include <string.h>
}

#include "pixfmt_convert.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace video_dec_node
{

class VideoDecoder
{
public:
    VideoDecoder(std::string &codec_name);

    int parseFrame(std::shared_ptr<sensor_msgs::msg::Image> image_msg, int pts_idx);

    void flushDecode();

    void decode(std::queue<std::shared_ptr<sensor_msgs::msg::Image>> &out_buffer);

    // void decode(std::shared_ptr<FILE> outfile);

    ~VideoDecoder();

private:
    std::shared_ptr<const AVCodec> m_codec;
    std::string m_codec_name;
    std::shared_ptr<AVCodecParserContext> m_parser;
    std::shared_ptr<AVCodecContext> m_ctx{nullptr};
    std::shared_ptr<AVFrame> m_frame;
    std::shared_ptr<AVPacket> m_pkt;

    std::shared_ptr<PixFmtConvert> m_convert;
};

}