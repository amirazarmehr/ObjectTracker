#include <stdio.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include "second.h"

int main(int argc, char** argv){
    if (argc < 2){
        fprintf(stderr, "Usage: %s <video-file>\n", argv[0]);
        return 1;
    }
    const char* path = argv[1];
    av_log_set_level(AV_LOG_ERROR);

    AVFormatContext* fmt = NULL;
    if (avformat_open_input(&fmt, path, NULL, NULL) < 0){
        fprintf(stderr, "Failed to open input: %s\n", path);
        return 1;
    }
    if (avformat_find_stream_info(fmt, NULL) < 0){
        fprintf(stderr, "Failed to find stream info\n");
        avformat_close_input(&fmt);
        return 1;
    }
    int vstream = av_find_best_stream(fmt, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
    if (vstream < 0){
        fprintf(stderr, "No video stream\n");
        avformat_close_input(&fmt);
        return 1;
    }
    AVStream* st = fmt->streams[vstream];
    const AVCodec* dec = avcodec_find_decoder(st->codecpar->codec_id);
    if (!dec){
        fprintf(stderr, "Decoder not found\n");
        avformat_close_input(&fmt);
        return 1;
    }
    AVCodecContext* ctx = avcodec_alloc_context3(dec);
    if (!ctx){
        fprintf(stderr, "Alloc codec ctx failed\n");
        avformat_close_input(&fmt);
        return 1;
    }
    if (avcodec_parameters_to_context(ctx, st->codecpar) < 0){
        fprintf(stderr, "parameters_to_context failed\n");
        avcodec_free_context(&ctx);
        avformat_close_input(&fmt);
        return 1;
    }
    if (avcodec_open2(ctx, dec, NULL) < 0){
        fprintf(stderr, "avcodec_open2 failed\n");
        avcodec_free_context(&ctx);
        avformat_close_input(&fmt);
        return 1;
    }

    run_video_session(fmt, ctx, vstream);

    avcodec_free_context(&ctx);
    avformat_close_input(&fmt);
    return 0;
}