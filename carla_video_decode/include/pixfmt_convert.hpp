#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>
#include <queue>
#include <cstdio>

extern "C" {
#include <libavutil/imgutils.h>
#include <libavutil/parseutils.h>
#include <libavutil/frame.h>
#include <libswscale/swscale.h>
}

#include "sensor_msgs/msg/image.hpp"

namespace video_dec_node
{

class PixFmtConvert
{
public:
    PixFmtConvert(enum AVPixelFormat dstFormat);

    std::shared_ptr<AVFrame> convertFormat(std::shared_ptr<AVFrame> src_frame);

    // void decode(std::shared_ptr<FILE> outfile);

    ~PixFmtConvert();

protected:
    void initConvertCtx(std::shared_ptr<AVFrame> src_frame);

private:
    bool m_ctxInitFlag{false};
    std::shared_ptr<AVFrame> m_dstFrame;
    struct SwsContext *m_sws_ctx;
    enum AVPixelFormat m_dst_pix_fmt;
};

}