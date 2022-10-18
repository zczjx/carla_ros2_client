#include "rclcpp/rclcpp.hpp"
#include "pixfmt_convert.hpp"
#include <iostream>

namespace video_dec_node
{

PixFmtConvert::PixFmtConvert(enum AVPixelFormat dstFormat)
    : m_dst_pix_fmt(dstFormat), m_ctxInitFlag(false)
{
    ;
}

void PixFmtConvert::initConvertCtx(std::shared_ptr<AVFrame> src_frame)
{
    std::string dst_format_name(av_get_pix_fmt_name(m_dst_pix_fmt));
    int ret;
    struct SwsContext *sws_ctx;

    m_dstFrame.reset(av_frame_alloc());

    if (nullptr == m_dstFrame)
    {
        std::cerr << "Could not allocate video frame" << std::endl;
        exit(1);
    }

    m_dstFrame->width = src_frame->width;
    m_dstFrame->height = src_frame->height;
    m_dstFrame->format = m_dst_pix_fmt;
    m_sws_ctx = sws_getContext(src_frame->width, src_frame->height, (enum AVPixelFormat) src_frame->format,
                            m_dstFrame->width, m_dstFrame->height, (enum AVPixelFormat)  m_dstFrame->format,
                            SWS_BILINEAR, NULL, NULL, NULL);

    if (NULL == m_sws_ctx)
    {
        std::cerr << "Impossible to create scale context for the conversion: " << dst_format_name << std::endl;
        exit(1);
    }

    ret = av_image_alloc(m_dstFrame->data, m_dstFrame->linesize,
                        m_dstFrame->width, m_dstFrame->height,
                        (enum AVPixelFormat)  m_dstFrame->format, 1);

    std::cerr << "image buffer size: " << std::to_string(ret) << std::endl;

    /* buffer is going to be written to rawvideo file, no alignment */
    if (ret < 0)
    {
        std::cerr << "Could not allocate destination image " << std::endl;
        exit(1);
    }
}

std::shared_ptr<AVFrame> PixFmtConvert::convertFormat(std::shared_ptr<AVFrame> src_frame)
{
    if (false == m_ctxInitFlag)
    {
        initConvertCtx(src_frame);
        m_ctxInitFlag = true;
    }

    sws_scale(m_sws_ctx, src_frame->data, src_frame->linesize,
            0, src_frame->height, m_dstFrame->data, m_dstFrame->linesize);

    return m_dstFrame;
}

PixFmtConvert::~PixFmtConvert()
{
    av_freep(&m_dstFrame->data[0]);
    sws_freeContext(m_sws_ctx);
}

}