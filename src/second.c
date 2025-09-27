/*
 * ZenoTrack — core tracker implementation
 *
 * Implements:
 *  - GRAY8 conversion with libswscale
 *  - Integral images: sum and sum of squares (1-based)
 *  - ZNCC template matching with coarse-to-fine search
 *  - 1D quadratic sub-pixel peak refinement
 *  - Simple motion model + adaptive search window
 *  - SDL2 rendering of the current frame and target boxes
 *
 * The public API is declared in include/second.h.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>
#include <SDL2/SDL.h>

#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>

#include "second.h"

/* ============================= Utilities ============================= */
static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* ====================== Runtime thresholds/steps ===================== */
typedef struct {
    double rho_good, rho_weak;
    double psr_good, psr_weak;
    int coarse_min, coarse_max;
} ThrParams;

static ThrParams g_thr = { 0.68, 0.52, 4.5, 2.8, 4, 8 };
static double g_ms_per_frame_ema = 0.0;
static int    g_last_eval_count  = 0;

/* Simple knobs exposed via header */
void tracker_set_thresholds(double rho_good, double rho_weak, double psr_good, double psr_weak){
    g_thr.rho_good = rho_good; g_thr.rho_weak = rho_weak;
    g_thr.psr_good = psr_good; g_thr.psr_weak = psr_weak;
}
void tracker_set_search_steps(int coarse_min, int coarse_max){
    g_thr.coarse_min = (coarse_min < 1) ? 1 : coarse_min;
    g_thr.coarse_max = (coarse_max < g_thr.coarse_min) ? g_thr.coarse_min : coarse_max;
}
void tracker_get_timing(double* ms_per_frame, int* n_evals_last){
    if (ms_per_frame)   *ms_per_frame   = g_ms_per_frame_ema;
    if (n_evals_last)   *n_evals_last   = g_last_eval_count;
}

/* ========================= Images + Integrals ======================== */
/* Compute 1-based integral images (sum and sum of squares).
 * We only clear the top row, and set the first column of each row to 0; the rest is overwritten.
 */
static void compute_integrals(Image* img){
    const int w = img->width, h = img->height, stride = w + 1;
    img->iistride = stride;
    const size_t need = (size_t)stride * (size_t)(h + 1);

    if (!img->ii || !img->ii2){
        free(img->ii); free(img->ii2);
        img->ii  = (uint64_t*)calloc(need, sizeof(uint64_t));
        img->ii2 = (uint64_t*)calloc(need, sizeof(uint64_t));
        if (!img->ii || !img->ii2) return;
    }

    /* zero the top row */
    memset(img->ii,  0, (size_t)(w + 1) * sizeof(uint64_t));
    memset(img->ii2, 0, (size_t)(w + 1) * sizeof(uint64_t));

    for (int y = 1; y <= h; ++y){
        const uint8_t* src = img->pixel + img->table[y-1];
        uint64_t rowSum = 0, rowSum2 = 0;

        /* first column is zero */
        const size_t row0 = (size_t)y * (size_t)stride;
        img->ii[row0]  = 0;
        img->ii2[row0] = 0;

        for (int x = 1; x <= w; ++x){
            const uint8_t v = src[x-1];
            rowSum  += v;
            rowSum2 += (uint64_t)v * (uint64_t)v;
            const size_t idx = row0 + (size_t)x;
            img->ii[idx]  = img->ii[idx - stride]  + rowSum;
            img->ii2[idx] = img->ii2[idx - stride] + rowSum2;
        }
    }
}

/* O(1) rectangle sums using the 1-based integral images */
static inline uint64_t rect_sum (const Image* img, int x, int y, int w, int h){
    const int s = img->iistride;
    const int x0 = x,     y0 = y;
    const int x1 = x + w, y1 = y + h;
    const size_t A = (size_t)y0 * s + x0;
    const size_t B = (size_t)y0 * s + x1;
    const size_t C = (size_t)y1 * s + x0;
    const size_t D = (size_t)y1 * s + x1;
    return img->ii[D] - img->ii[B] - img->ii[C] + img->ii[A];
}
static inline uint64_t rect_sum2(const Image* img, int x, int y, int w, int h){
    const int s = img->iistride;
    const int x0 = x,     y0 = y;
    const int x1 = x + w, y1 = y + h;
    const size_t A = (size_t)y0 * s + x0;
    const size_t B = (size_t)y0 * s + x1;
    const size_t C = (size_t)y1 * s + x0;
    const size_t D = (size_t)y1 * s + x1;
    return img->ii2[D] - img->ii2[B] - img->ii2[C] + img->ii2[A];
}

/* Create/update/free image wrappers around an FFmpeg AVFrame (GRAY8) */
Image* init_image_from_frame(const AVFrame* frame){
    if (!frame) return NULL;
    Image* img = (Image*)calloc(1, sizeof(Image));
    if (!img) return NULL;
    img->height = frame->height;
    img->width  = frame->width;
    img->size   = img->height * img->width;
    img->pixel  = (uint8_t*)malloc((size_t)img->size);
    img->table  = (int*)malloc(sizeof(int) * (size_t)img->height);
    if (!img->pixel || !img->table) { free(img->pixel); free(img->table); free(img); return NULL; }
    for (int row = 0; row < img->height; ++row) img->table[row] = row * img->width;

    for (int y = 0; y < img->height; ++y) {
        const uint8_t* src = frame->data[0] + (size_t)y * (size_t)frame->linesize[0];
        uint8_t* dst = img->pixel + img->table[y];
        memcpy(dst, src, (size_t)img->width);
    }
    compute_integrals(img);
    return img;
}
void update_image_from_frame(Image* img, const AVFrame* frame){
    if (!img || !frame) return;
    for (int y = 0; y < img->height; ++y) {
        const uint8_t* src = frame->data[0] + (size_t)y * (size_t)frame->linesize[0];
        uint8_t* dst = img->pixel + img->table[y];
        memcpy(dst, src, (size_t)img->width);
    }
    compute_integrals(img);
}
void free_image(Image* img){
    if (!img) return;
    free(img->pixel);
    free(img->table);
    free(img->ii);
    free(img->ii2);
    free(img);
}

/* ============================ GRAY via swscale ======================= */
/* Maintain a persistent swscale context and a GRAY8 frame */
static ConvCtx g_conv = {0};

/* Ensure the converter matches the source frame (recreate if needed) */
static int ensure_gray_converter(const AVFrame* src){
    if (!src) return -1;

    if (g_conv.sws && g_conv.w == src->width && g_conv.h == src->height &&
        g_conv.src_fmt == (enum AVPixelFormat)src->format) return 0;

    if (g_conv.sws)  { sws_freeContext(g_conv.sws); g_conv.sws = NULL; }
    if (g_conv.gray) { av_frame_free(&g_conv.gray); g_conv.gray = NULL; }

    g_conv.w = src->width; g_conv.h = src->height; g_conv.src_fmt = (enum AVPixelFormat)src->format;
    g_conv.sws = sws_getContext(g_conv.w, g_conv.h, g_conv.src_fmt,
                                g_conv.w, g_conv.h, AV_PIX_FMT_GRAY8,
                                SWS_FAST_BILINEAR, NULL, NULL, NULL);
    g_conv.gray = av_frame_alloc();
    if (!g_conv.sws || !g_conv.gray) return -1;

    g_conv.gray->format = AV_PIX_FMT_GRAY8;
    g_conv.gray->width  = g_conv.w;
    g_conv.gray->height = g_conv.h;
    if (av_frame_get_buffer(g_conv.gray, 32) < 0) return -1;
    return 0;
}

/* Convert an arbitrary source frame to GRAY8 using swscale */
static inline AVFrame* to_gray8(const AVFrame* src){
    if (ensure_gray_converter(src) != 0) return NULL;
    sws_scale(g_conv.sws, (const uint8_t* const*)src->data, src->linesize, 0, g_conv.h,
              g_conv.gray->data, g_conv.gray->linesize);
    return g_conv.gray;
}
static void free_gray_converter(void){
    if (g_conv.sws) sws_freeContext(g_conv.sws);
    if (g_conv.gray) av_frame_free(&g_conv.gray);
    memset(&g_conv, 0, sizeof(g_conv));
}

/* =============================== NCC/ZNCC ============================ */
/* Online stats (Welford) */
static inline void stats_add(Stats* s, double v){ s->n++; double d=v-s->mean; s->mean += d/s->n; s->M2 += d*(v-s->mean); }
static inline double stats_std(const Stats* s){ return (s->n>1)? sqrt(s->M2/(s->n-1)) : 0.0; }

/* Evaluate ZNCC at position (x,y) for target t.
 * Numerically safe: clamp small variances; branch-free inner loop for vectorization.
 */
static double zncc_at(const Image* img, const Target* t, int x, int y){
    const int w = t->width, h = t->height, N = t->size;
    const uint64_t S  = rect_sum(img, x, y, w, h);
    const uint64_t S2 = rect_sum2(img, x, y, w, h);
    const double varI = (double)S2 - ((double)S * (double)S) / (double)N;
    if (varI <= 1e-6) return -1e9;

    double dot = 0.0;
    for (int r=0; r<h; ++r){
        const uint8_t* ip = img->pixel + img->table[y + r] + x;
        const float*   tp = t->templ   + t->table[r];
        for (int c=0; c<w; ++c){
            dot += (double)ip[c] * (double)tp[c];
        }
    }
    const double denom = sqrt(varI) * sqrt(t->sumsqT);
    return dot / (denom + 1e-12);
}

/* Coarse-to-fine search around (cx,cy).
 * Returns the number of ZNCC evaluations and writes out best position and score.
 */
static int match_template_fast_zncc(const Image* img, const Target* trg,
                                    int* bestX, int* bestY,
                                    int cx, int cy, int radius, int step,
                                    double* bestRhoOut, Stats* outStats){
    int evals = 0;
    const int limitX = img->width  - trg->width;
    const int limitY = img->height - trg->height;
    if (limitX < 0 || limitY < 0) { if(bestRhoOut) *bestRhoOut=-1e9; return evals; }

    const int x0 = clampi(cx - radius, 0, limitX);
    const int y0 = clampi(cy - radius, 0, limitY);
    const int x1 = clampi(cx + radius, 0, limitX);
    const int y1 = clampi(cy + radius, 0, limitY);

    double bestRho = -1e9;
    int bx = x0, by = y0;
    Stats st = (Stats){0};

    const int stp = clampi(step, g_thr.coarse_min, g_thr.coarse_max);

    for (int i=y0; i<=y1; i+=stp){
        for (int j=x0; j<=x1; j+=stp){
            const double rho = zncc_at(img, trg, j, i);
            stats_add(&st, rho);
            if (rho > bestRho){ bestRho = rho; bx=j; by=i; }
            ++evals;
        }
    }

    /* 1-step refinement around the coarse peak (dense) */
    const int refR = (stp>1 ? stp : 1);
    const int rx0 = clampi(bx - refR, 0, limitX);
    const int ry0 = clampi(by - refR, 0, limitY);
    const int rx1 = clampi(bx + refR, 0, limitX);
    const int ry1 = clampi(by + refR, 0, limitY);

    for (int i=ry0; i<=ry1; ++i){
        for (int j=rx0; j<=rx1; ++j){
            const double rho = zncc_at(img, trg, j, i);
            stats_add(&st, rho);
            if (rho > bestRho){ bestRho = rho; bx=j; by=i; }
            ++evals;
        }
    }

    *bestX = bx; *bestY = by;
    if (bestRhoOut) *bestRhoOut = bestRho;
    if (outStats)   *outStats   = st;
    return evals;
}

/* 1D quadratic sub-pixel offset from three samples: f(-1), f(0), f(+1).
 * Returns an offset in [-0.5, 0.5] (approximately) if the parabola is well‑posed.
 */
static inline double quad_subpix(double fm1, double f0, double fp1){
    const double denom = (fm1 - 2.0*f0 + fp1);
    if (fabs(denom) < 1e-12) return 0.0;
    return 0.5 * (fm1 - fp1) / denom;
}

/* ============================== SDL helpers ========================== */
static uint8_t *g_uv = NULL; static int g_uw=0, g_uh=0;

/* Allocate/update a neutral UV plane for IYUV uploading (we only display Y) */
static void ensure_uv(int w, int h){
    const int uw = (w+1)/2, uh = (h+1)/2;
    if (!g_uv || g_uw!=uw || g_uh!=uh){
        free(g_uv);
        g_uv = (uint8_t*)malloc((size_t)uw*uh*2);
        if (g_uv) memset(g_uv, 128, (size_t)uw*uh*2); /* neutral chroma */
        g_uw = uw; g_uh = uh;
    }
}

/* Initialize SDL window/renderer/texture given an image size */
static int sdl_init_with_image(const Image* img, SdlCtx* ctx){
    if (SDL_Init(SDL_INIT_VIDEO) < 0){
        fprintf(stderr,"SDL init: %s\n", SDL_GetError());
        return -1;
    }
    ctx->win_w = img->width  / 2 > 0 ? img->width  / 2 : 1;
    ctx->win_h = img->height / 2 > 0 ? img->height / 2 : 1;
    ctx->sx = (double)img->width  / ctx->win_w;
    ctx->sy = (double)img->height / ctx->win_h;
    ctx->window = SDL_CreateWindow("Tracker (ZNCC, Multi-Target)",
                        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                        ctx->win_w, ctx->win_h, SDL_WINDOW_SHOWN);
    if (!ctx->window){ fprintf(stderr,"Window: %s\n", SDL_GetError()); SDL_Quit(); return -1; }
    ctx->renderer = SDL_CreateRenderer(ctx->window, -1, SDL_RENDERER_ACCELERATED);
    if (!ctx->renderer){ fprintf(stderr,"Renderer: %s\n", SDL_GetError()); SDL_DestroyWindow(ctx->window); SDL_Quit(); return -1; }
    ctx->texture = SDL_CreateTexture(ctx->renderer, SDL_PIXELFORMAT_IYUV, SDL_TEXTUREACCESS_STREAMING, img->width, img->height);
    if (!ctx->texture){ SDL_DestroyRenderer(ctx->renderer); SDL_DestroyWindow(ctx->window); SDL_Quit(); return -1; }
    ensure_uv(img->width, img->height);
    return 0;
}
static void sdl_cleanup(SdlCtx* ctx){
    if (ctx->texture)  SDL_DestroyTexture(ctx->texture);
    if (ctx->renderer) SDL_DestroyRenderer(ctx->renderer);
    if (ctx->window)   SDL_DestroyWindow(ctx->window);
    SDL_Quit();
    free(g_uv); g_uv=NULL; g_uw=g_uh=0;
}

/* Upload Y plane, draw dragging rect (if any) and target boxes */
static void draw_frame_multi(SdlCtx* ctx, const Image* img, SDL_Rect* drag_win, Target** arr, int n){
    ensure_uv(img->width, img->height);
    const uint8_t* Y = img->pixel;          const int Ypitch  = img->width;
    const uint8_t* U = g_uv;                const int Upitch  = g_uw;
    const uint8_t* V = g_uv + (size_t)g_uw*(size_t)g_uh;    const int Vpitch  = g_uw;
    SDL_UpdateYUVTexture(ctx->texture, NULL, Y, Ypitch, U, Upitch, V, Vpitch);

    SDL_SetRenderDrawColor(ctx->renderer, 0,0,0,255);
    SDL_RenderClear(ctx->renderer);
    SDL_RenderCopy(ctx->renderer, ctx->texture, NULL, NULL);

    if (drag_win && drag_win->w>0 && drag_win->h>0){
        SDL_SetRenderDrawColor(ctx->renderer, 0,255,0,255);
        SDL_RenderDrawRect(ctx->renderer, drag_win);
    }

    for (int i=0; i<n; ++i){
        Target* t = arr[i];
        SDL_Rect r = { (int)(t->x / ctx->sx), (int)(t->y / ctx->sy), (int)(t->width / ctx->sx), (int)(t->height / ctx->sy) };
        if (t->status == TSTAT_GOOD) SDL_SetRenderDrawColor(ctx->renderer, 0,255,0,255);
        else if (t->status == TSTAT_WEAK) SDL_SetRenderDrawColor(ctx->renderer, 255,255,0,255);
        else SDL_SetRenderDrawColor(ctx->renderer, 255,0,0,255);
        SDL_RenderDrawRect(ctx->renderer, &r);
    }
    SDL_RenderPresent(ctx->renderer);
}

/* ============================ Targets container ===================== */
static void tl_init(TargetList* tl){ tl->v=NULL; tl->n=0; tl->cap=0; }
static void tl_clear(TargetList* tl){
    for(int i=0;i<tl->n;i++){ Target* t = tl->v[i]; if (t){ free(t->templ); free(t->table); free(t);} }
    free(tl->v); tl->v=NULL; tl->n=tl->cap=0;
}
static void tl_push(TargetList* tl, Target* t){
    if (!t) return;
    if (tl->n == tl->cap){
        const int nc = tl->cap ? tl->cap*2 : 4;
        Target** nv = (Target**)realloc(tl->v, sizeof(Target*)*(size_t)nc);
        if (!nv) { free(t->templ); free(t->table); free(t); return; }
        tl->v = nv; tl->cap = nc;
    }
    tl->v[tl->n++] = t;
}

/* ============================ Target lifecycle ====================== */
/* Initialize a target from an image ROI. A tiny border is cropped to reduce background. */
static Target* init_target(const Image* img, int x, int y, int w, int h){
    if (!img) return NULL;
    x = clampi(x, 0, img->width-1);
    y = clampi(y, 0, img->height-1);
    if (x + w > img->width)  w = img->width  - x;
    if (y + h > img->height) h = img->height - y;
    if (w < 6 || h < 6) return NULL;

    /* Optional: trim a small border to focus on the object body */
    const int border = 2;
    if (w > 2*border && h > 2*border) {
        x += border; y += border; w -= 2*border; h -= 2*border;
    }

    Target* t = (Target*)calloc(1, sizeof(Target));
    if (!t) return NULL;
    t->x = x; t->y = y; t->width = w; t->height = h; t->size = w*h;
    t->templ = (float*)malloc(sizeof(float) * (size_t)t->size);
    t->table = (int*)  malloc(sizeof(int)   * (size_t)h);
    if (!t->templ || !t->table) { free(t->templ); free(t->table); free(t); return NULL; }
    for (int r=0;r<h;r++) t->table[r] = r*w;

    /* Build zero-mean template T' */
    const uint64_t S  = rect_sum(img, x, y, w, h);
    const double mean = (double)S / (double)(t->size);
    t->sumsqT = 0.0;
    for (int r=0;r<h;r++){
        const int ib = img->table[y + r] + x;
        const int tb = t->table[r];
        for (int c=0;c<w;c++){
            const float v = (float)img->pixel[ib + c] - (float)mean;
            t->templ[tb + c] = v;
            t->sumsqT += (double)v * (double)v;
        }
    }
    if (t->sumsqT < 1e-6) t->sumsqT = 1e-6;

    t->cx = x + 0.5*w; t->cy = y + 0.5*h;
    t->vx = t->vy = 0.0;
    t->R = 96; t->step = 6;
    t->lost = 0; t->last_rho = 0.0; t->status = TSTAT_GOOD;
    return t;
}

/* Blend the template with the current patch (small alpha to avoid drift). */
static void update_target_model(Target* t, const Image* img, int newX, int newY, double alpha){
    if (!t || !img) return;
    if (newX < 0 || newY < 0 || newX + t->width > img->width || newY + t->height > img->height) return;
    const uint64_t S  = rect_sum(img, newX, newY, t->width, t->height);
    const double mean = (double)S / (double)t->size;

    double sumsq = 0.0;
    for (int r=0;r<t->height;r++){
        const int ib = img->table[newY + r] + newX;
        const int tb = t->table[r];
        for (int c=0;c<t->width;c++){
            const float pv = (float)img->pixel[ib + c] - (float)mean;  /* patch' */
            const float tv = t->templ[tb + c];
            const float nv = (float)((1.0 - alpha)*tv + alpha*pv);
            t->templ[tb + c] = nv;
            sumsq += (double)nv * (double)nv;
        }
    }
    t->sumsqT = (sumsq < 1e-6) ? 1e-6 : sumsq;
    t->x = newX; t->y = newY;
    t->cx = newX + 0.5*t->width;
    t->cy = newY + 0.5*t->height;
}

/* =========================== Per-frame processing =================== */
/* Sleep only the remaining budget (to keep real-time pace) and track EMA. */
static void sleep_remainder(double budget_ms, uint64_t t_start){
    const uint64_t t_end = SDL_GetPerformanceCounter();
    const double elapsed_ms = 1000.0 * (double)(t_end - t_start) / (double)SDL_GetPerformanceFrequency();
    const double left = budget_ms - elapsed_ms;
    if (g_ms_per_frame_ema == 0.0) g_ms_per_frame_ema = elapsed_ms;
    else g_ms_per_frame_ema = 0.9*g_ms_per_frame_ema + 0.1*elapsed_ms;

    if (left > 0.5) SDL_Delay((Uint32)left);
}

/* Process a decoded frame: convert to GRAY, update image, handle events, track, render. */
static void process_frame(SdlCtx* sdl,
                          Image** pimg,
                          TargetList* tl,
                          SDL_Rect* dragRect,
                          int* dragging,
                          int* running,
                          const AVFrame* srcFrame,
                          double budget_ms){

    const uint64_t t0 = SDL_GetPerformanceCounter();

    /* Convert to GRAY8 */
    AVFrame* f = to_gray8(srcFrame);
    if (!f){ *running = 0; return; }

    /* Init/update image */
    if (!*pimg) {
        *pimg = init_image_from_frame(f);
        if (!*pimg || sdl_init_with_image(*pimg, sdl) != 0) { *running = 0; return; }
    } else {
        update_image_from_frame(*pimg, f);
    }
    Image* img = *pimg;

    /* Input events (mouse/keyboard) */
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        if (e.type == SDL_QUIT) *running = 0;
        if (e.type == SDL_KEYDOWN) {
            if (e.key.keysym.sym == SDLK_c) { tl_clear(tl); }          /* clear targets */
            else if (e.key.keysym.sym == SDLK_q) { *running = 0; }     /* quit */
        }
        if (e.type == SDL_MOUSEBUTTONDOWN) {
            if (e.button.button == SDL_BUTTON_LEFT) {
                *dragging = 1;
                const int mx = (int)(e.button.x * sdl->sx);
                const int my = (int)(e.button.y * sdl->sy);
                *dragRect = (SDL_Rect){ mx, my, 0, 0 };
            } else if (e.button.button == SDL_BUTTON_RIGHT){
                tl_clear(tl);                                          /* clear all targets */
            }
        }
        if (e.type == SDL_MOUSEMOTION && *dragging) {
            const int mx2 = (int)(e.motion.x * sdl->sx);
            const int my2 = (int)(e.motion.y * sdl->sy);
            dragRect->w = mx2 - dragRect->x;
            dragRect->h = my2 - dragRect->y;
        }
        if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT) {
            *dragging = 0;
            const int ux = (int)(e.button.x * sdl->sx);
            const int uy = (int)(e.button.y * sdl->sy);
            SDL_Rect roi = *dragRect;
            roi.w = ux - roi.x; roi.h = uy - roi.y;
            if (roi.w < 0) { roi.x += roi.w; roi.w = -roi.w; }
            if (roi.h < 0) { roi.y += roi.h; roi.h = -roi.h; }
            if (roi.w>0 && roi.h>0) {
                Target* t = init_target(img, roi.x, roi.y, roi.w, roi.h);
                if (t) tl_push(tl, t);
            }
        }
    }

    /* Track each target */
    static int frame_id = 0; frame_id++;
    int evals_total = 0;

    for (int k=0; k<tl->n; ++k){
        Target* t = tl->v[k];

        /* Predict center with a simple constant-velocity model */
        const double pcx = t->cx + t->vx;
        const double pcy = t->cy + t->vy;
        const int pred_x = clampi((int)floor(pcx - 0.5*t->width),  0, img->width  - t->width);
        const int pred_y = clampi((int)floor(pcy - 0.5*t->height), 0, img->height - t->height);

        double bestRho = -1.0;
        Stats st = (Stats){0};
        int bestX = t->x, bestY = t->y;

        const int evals = match_template_fast_zncc(img, t, &bestX, &bestY, pred_x, pred_y, t->R, t->step, &bestRho, &st);
        evals_total += evals;

        /* Sub-pixel refine around the integer peak (reuse bestRho as the center) */
        const double cx0 = zncc_at(img, t, clampi(bestX-1,0,img->width-t->width), bestY);
        const double cx1 = bestRho;
        const double cx2 = zncc_at(img, t, clampi(bestX+1,0,img->width-t->width), bestY);
        const double dx = quad_subpix(cx0, cx1, cx2);

        const double cy0 = zncc_at(img, t, bestX, clampi(bestY-1,0,img->height-t->height));
        const double cy1 = bestRho;
        const double cy2 = zncc_at(img, t, bestX, clampi(bestY+1,0,img->height-t->height));
        const double dy = quad_subpix(cy0, cy1, cy2);

        /* Confidence via PSR */
        const double std = stats_std(&st);
        const double psr = (std > 1e-6) ? (bestRho - st.mean) / std : 0.0;

        TrackStatus status;
        if (bestRho >= g_thr.rho_good && psr >= g_thr.psr_good) status = TSTAT_GOOD;
        else if (bestRho >= g_thr.rho_weak && psr >= g_thr.psr_weak) status = TSTAT_WEAK;
        else status = TSTAT_LOST;

        /* Update motion */
        const double new_cx = (double)bestX + 0.5*t->width + dx;
        const double new_cy = (double)bestY + 0.5*t->height + dy;
        const double dvx    = new_cx - t->cx;
        const double dvy    = new_cy - t->cy;
        t->vx = 0.8*t->vx + 0.2*dvx;
        t->vy = 0.8*t->vy + 0.2*dvy;

        /* Adapt search window based on confidence and motion */
        if (status == TSTAT_LOST) {
            t->R   = 200;                       /* wide to re-acquire */
            t->step = g_thr.coarse_max;         /* faster, coarse sampling */
        } else if (status == TSTAT_WEAK) {
            t->R   = 140;
            t->step = (g_thr.coarse_min + g_thr.coarse_max) / 2;
        } else {
            const double spd2 = dvx*dvx + dvy*dvy;
            if (spd2 < 9.0)       { t->R = 48;  t->step = g_thr.coarse_min; }
            else if (spd2 < 100)  { t->R = 96;  t->step = (g_thr.coarse_min + g_thr.coarse_max)/2; }
            else                  { t->R = 160; t->step = g_thr.coarse_max; }
        }

        /* Conservative model update to avoid locking onto distractors */
        const double mean_new = (double)rect_sum(img, bestX, bestY, t->width, t->height) / (double)t->size;
        const double mean_old = (double)rect_sum(img, t->x,   t->y,   t->width, t->height) / (double)t->size;
        const double dMean    = fabs(mean_new - mean_old);
        const double move2    = dvx*dvx + dvy*dvy;
        const int ok_to_update = (status == TSTAT_GOOD) && (dMean < 12.0) && (move2 < 200.0);

        if (ok_to_update && ((frame_id % 2) == 0)) {
            update_target_model(t, img, bestX, bestY, 0.08);
            t->lost = 0;
        } else {
            t->x  = bestX; t->y  = bestY;
            t->cx = new_cx; t->cy = new_cy;
            if (status == TSTAT_LOST) t->lost++;
        }

        t->status   = status;
        t->last_rho = bestRho;
    }

    /* Render overlay */
    const SDL_Rect drag_win = (SDL_Rect){
        (int)(dragRect->x / sdl->sx),
        (int)(dragRect->y / sdl->sy),
        (int)(dragRect->w / sdl->sx),
        (int)(dragRect->h / sdl->sy)
    };
    draw_frame_multi(sdl, img, *dragging ? (SDL_Rect*)&drag_win : NULL, tl->v, tl->n);

    g_last_eval_count = evals_total;

    if (budget_ms > 0.0) sleep_remainder(budget_ms, t0);
}

/* =============================== Session ============================= */
/* Main video loop: read packets/frames, send to tracker. */
void run_video_session(AVFormatContext* fmt, AVCodecContext* codec_ctx, int stream_index) {
    SdlCtx sdl; memset(&sdl, 0, sizeof(sdl));
    Image* img = NULL;
    TargetList tl; tl_init(&tl);
    SDL_Rect dragRect = (SDL_Rect){0,0,0,0};
    int dragging = 0;
    int running = 1;

    AVRational fr = av_guess_frame_rate(fmt, fmt->streams[stream_index], NULL);
    const double budget_ms = (fr.num && fr.den) ? 1000.0 * fr.den / fr.num : 0.0;

    AVPacket* pkt = av_packet_alloc();
    AVFrame* frame = av_frame_alloc();
    if (!pkt || !frame) { if(pkt) av_packet_free(&pkt); if(frame) av_frame_free(&frame); return; }

    while (running && av_read_frame(fmt, pkt) >= 0) {
        if (pkt->stream_index == stream_index) {
            if (avcodec_send_packet(codec_ctx, pkt) == 0) {
                while (avcodec_receive_frame(codec_ctx, frame) == 0) {
                    process_frame(&sdl, &img, &tl, &dragRect, &dragging, &running, frame, budget_ms);
                    if (!running) break;
                }
            }
        }
        av_packet_unref(pkt);
    }

    /* Drain decoder */
    if (running) {
        avcodec_send_packet(codec_ctx, NULL);
        while (avcodec_receive_frame(codec_ctx, frame) == 0) {
            process_frame(&sdl, &img, &tl, &dragRect, &dragging, &running, frame, budget_ms);
            if (!running) break;
        }
    }

    tl_clear(&tl);
    if (img) free_image(img);
    sdl_cleanup(&sdl);
    av_frame_free(&frame);
    av_packet_free(&pkt);
    free_gray_converter();
}
