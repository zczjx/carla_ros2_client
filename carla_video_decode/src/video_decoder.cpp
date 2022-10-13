#include "rclcpp/rclcpp.hpp"
#include "video_encoder.hpp"
#include <iostream>

namespace video_enc_node
{

VideoEncoder::VideoEncoder(std::string &codec_name)
    : m_codec_name(codec_name)
{
    int ret;
    /* find the mpeg1video encoder */
    m_codec.reset(avcodec_find_encoder_by_name(m_codec_name.c_str()));

    if (nullptr == m_codec)
    {
        std::cerr << "Codec: " << m_codec_name << "not found" << std::endl;
        exit(1);
    }

    m_ctx.reset(avcodec_alloc_context3(m_codec.get()));

    if (nullptr == m_ctx)
    {
        std::cerr << "Could not allocate video codec context" << std::endl;
        exit(1);
    }

    m_pkt.reset(av_packet_alloc());
    if (nullptr == m_pkt)
        exit(1);

    /* put sample parameters */
    m_ctx->bit_rate = 400000;
    /* resolution must be a multiple of two */
    m_ctx->width = 800;
    m_ctx->height = 600;
    /* frames per second */
    m_ctx->time_base = (AVRational){1, 20};
    m_ctx->framerate = (AVRational){20, 1};

    /* emit one intra frame every ten frames
     * check frame pict_type before passing frame
     * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
     * then gop_size is ignored and the output of encoder
     * will always be I frame irrespective to gop_size
     */
    m_ctx->gop_size = 10;
    m_ctx->max_b_frames = 1;
    m_ctx->pix_fmt = AV_PIX_FMT_BGRA;

    if (m_codec->id == AV_CODEC_ID_H264)
        av_opt_set(m_ctx->priv_data, "preset", "slow", 0);

    /* open it */
    ret = avcodec_open2(m_ctx.get(), m_codec.get(), NULL);

    if (ret < 0) {
        std::cerr << "Could not open codec: " << m_codec_name << std::endl;
        exit(1);
    }


    m_frame.reset(av_frame_alloc());
    if (nullptr == m_frame)
    {
        std::cerr << "Could not allocate video frame" << std::endl;
        exit(1);
    }

    m_frame->format = m_ctx->pix_fmt;
    m_frame->width  = m_ctx->width;
    m_frame->height = m_ctx->height;

    ret = av_frame_get_buffer(m_frame.get(), 0);
    if (ret < 0)
    {
        std::cerr << "Could not allocate the video frame data" << std::endl;
        exit(1);
    }
}

std::shared_ptr<AVFrame> VideoEncoder::fillinFrame(std::shared_ptr<sensor_msgs::msg::Image> image_msg, int pts_idx)
{
    int ret = av_frame_make_writable(m_frame.get());

    if (ret < 0)
        exit(1);

    size_t bytes = 4 * m_frame->width * m_frame->height;
    memcpy(m_frame->data[0], image_msg->data.data(), bytes);
    m_frame->pts = pts_idx;

    return m_frame;

}

void VideoEncoder::flushEncode(std::shared_ptr<FILE> outfile)
{
    encode(nullptr, outfile);
    fclose(outfile.get());
}


void VideoEncoder::encode(std::shared_ptr<AVFrame> frame, std::queue<std::shared_ptr<sensor_msgs::msg::Image>> &out_buffer)
{
    int ret;

    ret = avcodec_send_frame(m_ctx.get(), frame.get());

    if (ret < 0)
    {
        std::cerr << "Error sending a frame for encoding" << std::endl;
        exit(1);
    }

    while (ret >= 0)
    {
        ret = avcodec_receive_packet(m_ctx.get(), m_pkt.get());

        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            return;
        else if (ret < 0)
        {
            std::cerr << "Error during encoding" << std::endl;
            exit(1);
        }

        auto encode_msg = std::make_shared<sensor_msgs::msg::Image>();
        encode_msg->header.frame_id = "video_enc/image_h264";
        encode_msg->encoding = m_codec_name;
        encode_msg->step = m_pkt->pts;
        encode_msg->data.insert(encode_msg->data.end(), &m_pkt->data[0], &m_pkt->data[m_pkt->size]);
        out_buffer.push(encode_msg);
        av_packet_unref(m_pkt.get());
    }
}


void VideoEncoder::encode(std::shared_ptr<AVFrame> frame, std::shared_ptr<FILE> outfile)
{
    int ret;

    ret = avcodec_send_frame(m_ctx.get(), frame.get());

    if (ret < 0)
    {
        std::cerr << "Error sending a frame for encoding" << std::endl;
        exit(1);
    }

    while (ret >= 0)
    {
        ret = avcodec_receive_packet(m_ctx.get(), m_pkt.get());

        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            return;
        else if (ret < 0)
        {
            std::cerr << "Error during encoding" << std::endl;
            exit(1);
        }

        printf("Write packet %3"PRId64" (size=%5d)\n", m_pkt->pts, m_pkt->size);
        fwrite(m_pkt->data, 1, m_pkt->size, outfile.get());
        av_packet_unref(m_pkt.get());
    }

}

VideoEncoder::~VideoEncoder()
{
    AVCodecContext *ctx = m_ctx.get();
    AVFrame *frame = m_frame.get();
    AVPacket *pkt = m_pkt.get();


    avcodec_free_context(&ctx);
    av_frame_free(&frame);
    av_packet_free(&pkt);
}

}