#ifndef SECOND_H
#define SECOND_H

#include <stdint.h>
#include <SDL2/SDL.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>

/**
 * @brief Tracking status (used for UI coloring and debugging).
 */
typedef enum {
    TSTAT_LOST = 0,   /**< Target not confidently matched in the current frame (red).   */
    TSTAT_WEAK = 1,   /**< Weak/ambiguous match (yellow).                                */
    TSTAT_GOOD = 2    /**< Confident match (green).                                       */
} TrackStatus;

/**
 * @brief Single-channel (GRAY8) image plus 1-based integral images.
 *
 * Integral images are computed with an extra top row and left column of zeros
 * (i.e., indices range [0..width] x [0..height]). This enables O(1) sums over
 * rectangles via the standard inclusion-exclusion formula.
 */
typedef struct Image {
    int width, height, size;
    uint8_t*  pixel;   /* Y plane data, row-major, width*height bytes */
    int*      table;   /* row offsets in @pixel (table[y] = y*width)  */
    /* 1-based integral images */
    uint64_t* ii;      /* integral of Y                              */
    uint64_t* ii2;     /* integral of Y^2                            */
    int       iistride;
} Image;

/**
 * @brief SDL context used for rendering the current frame and overlays.
 */
typedef struct SdlCtx {
    SDL_Window*   window;
    SDL_Renderer* renderer;
    SDL_Texture*  texture;
    int    win_w, win_h;
    double sx, sy;     /* image->window scale factors */
} SdlCtx;

/**
 * @brief Grayscale conversion context (libswscale).
 * Keeps a persistent SwsContext + an AVFrame in GRAY8.
 */
typedef struct ConvCtx {
    struct SwsContext* sws;
    AVFrame* gray;
    int w, h;
    enum AVPixelFormat src_fmt;
} ConvCtx;

/**
 * @brief Online mean/variance accumulator for PSR statistics.
 * Welford's algorithm: M2 is the sum of squared deviations.
 */
typedef struct {
    int n;
    double mean, M2;
} Stats;

/**
 * @brief Track target (template and motion state).
 *
 * Template is zero-mean and stored in row-major floats (`templ`).
 * `table[r] = r*width` for fast row addressing.
 */
typedef struct Target {
    /* ROI in the full-resolution image */
    int x, y, width, height, size;
    /* Zero-mean template T' */
    float* templ;
    int*   table;      /* row offsets in @templ */
    double sumsqT;     /* sum of squares of T'  */

    /* Motion + state */
    double cx, cy;     /* center (px) */
    double vx, vy;     /* velocity (px/frame) */
    int    R, step;    /* search radius and coarse step */
    int    lost;       /* consecutive frames in LOST state */
    double last_rho;   /* last ZNCC score */
    TrackStatus status;
} Target;

/**
 * @brief Dynamic array of targets.
 */
typedef struct TargetList {
    Target** v;
    int n, cap;
} TargetList;

/* --------------------------- Public API --------------------------- */

/**
 * @brief Run a video tracking session: read frames, track, render.
 * @param fmt          Opened AVFormatContext (input)
 * @param ctx          Opened AVCodecContext for the chosen video stream
 * @param stream_index Index of the video stream in @fmt
 */
void    run_video_session(AVFormatContext* fmt, AVCodecContext* ctx, int stream_index);

/* Runtime tuning */
void    tracker_set_thresholds(double rho_good, double rho_weak, double psr_good, double psr_weak);
void    tracker_set_search_steps(int coarse_min, int coarse_max);
/**
 * @brief Expose a moving average of frame processing time and the last evaluation count.
 * @param ms_per_frame_ema  Out: EMA of ms/frame
 * @param n_evals_last      Out: scalar template evaluations in the last frame
 */
void    tracker_get_timing(double* ms_per_frame_ema, int* n_evals_last);

/* Image helpers */
Image*  init_image_from_frame(const AVFrame* frame);
void    update_image_from_frame(Image* img, const AVFrame* frame);
void    free_image(Image* img);

#endif /* SECOND_H */
