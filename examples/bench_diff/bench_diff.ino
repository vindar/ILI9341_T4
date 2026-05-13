/*
 * ILI9341_T4 DiffBuff benchmark and correctness check.
 *
 * This sketch does not use the display or SPI. It benchmarks only the
 * DiffBuff::computeDiff() phase and checks the generated raw diff stream.
 */

#include <Arduino.h>
#include <ILI9341_T4.h>

using namespace ILI9341_T4;

static constexpr int LX = DiffBuffBase::LX;
static constexpr int LY = DiffBuffBase::LY;
static constexpr int NB_PIXELS = LX * LY;

static constexpr int WARMUP = 8;
static constexpr int ITER = 60;

static constexpr int LARGE_DIFF_SIZE = 60000;
static constexpr int SMALL_DIFF_SIZE = 512;

DMAMEM uint16_t fb_old[NB_PIXELS + 1];
DMAMEM uint16_t fb_new[NB_PIXELS + 1];

DiffBuffStatic<LARGE_DIFF_SIZE> diff_large;
DiffBuffStatic<SMALL_DIFF_SIZE> diff_small;

volatile uint32_t sink = 0;

enum Scenario : uint8_t
{
    SCN_NO_CHANGE,
    SCN_ALL_CHANGE,
    SCN_SPARSE_64,
    SCN_FRAGMENTED_8,
    SCN_LOCAL_RECT,
    SCN_BLOCKS,
    SCN_MASK_NOISE,
};

struct ScenarioDef
{
    const char* name;
    Scenario scenario;
    uint16_t compare_mask;
};

struct BenchResult
{
    uint64_t total_us;
    uint32_t min_us;
    uint32_t max_us;
    int last_size;
    uint32_t overflow_count;
};

static const ScenarioDef scenarios[] =
{
    { "none",       SCN_NO_CHANGE,     0 },
    { "all",        SCN_ALL_CHANGE,    0 },
    { "sparse64",   SCN_SPARSE_64,     0 },
    { "frag8",      SCN_FRAGMENTED_8,  0 },
    { "local",      SCN_LOCAL_RECT,    0 },
    { "blocks",     SCN_BLOCKS,        0 },
    { "mask_noise", SCN_MASK_NOISE,    0xfffe },
};

static const int orientations[] =
{
    DiffBuffBase::PORTRAIT_240x320,
    DiffBuffBase::LANDSCAPE_320x240,
    DiffBuffBase::PORTRAIT_240x320_FLIPPED,
    DiffBuffBase::LANDSCAPE_320x240_FLIPPED,
};

static const int gaps[] = { 1, 4, 16 };

static const char* orientationName(int orientation)
{
    switch (orientation)
    {
    case DiffBuffBase::PORTRAIT_240x320: return "rot0";
    case DiffBuffBase::LANDSCAPE_320x240: return "rot90";
    case DiffBuffBase::PORTRAIT_240x320_FLIPPED: return "rot180";
    case DiffBuffBase::LANDSCAPE_320x240_FLIPPED: return "rot270";
    }
    return "?";
}

static uint32_t sourceWidth(int orientation)
{
    return (orientation & 1) ? LY : LX;
}

static uint32_t sourceHeight(int orientation)
{
    return (orientation & 1) ? LX : LY;
}

static uint16_t rgb565(uint32_t r, uint32_t g, uint32_t b)
{
    return (uint16_t)(((r & 31) << 11) | ((g & 63) << 5) | (b & 31));
}

static uint16_t basePixel(uint32_t x, uint32_t y)
{
    return rgb565((x * 3 + y * 5 + 7), (x * 7 + y * 11 + 13), (x * 13 + y * 17 + 19));
}

static uint16_t scenarioPixel(Scenario scenario, uint32_t x, uint32_t y, uint32_t w, uint32_t h, bool new_frame)
{
    const uint32_t index = x + (w * y);
    const uint16_t base = basePixel(x, y);
    if (!new_frame) return base;

    switch (scenario)
    {
    case SCN_NO_CHANGE:
        return base;
    case SCN_ALL_CHANGE:
        return (uint16_t)(base ^ 0xffff);
    case SCN_SPARSE_64:
        return ((index & 63) == 0) ? (uint16_t)(base ^ 0xffff) : base;
    case SCN_FRAGMENTED_8:
        return ((index & 7) == 0) ? (uint16_t)(base ^ 0xffff) : base;
    case SCN_LOCAL_RECT:
        if ((x >= w / 4) && (x < (3 * w) / 4) && (y >= h / 3) && (y < (2 * h) / 3))
        {
            return (uint16_t)(base ^ 0xffff);
        }
        return base;
    case SCN_BLOCKS:
        return ((((x >> 4) + (y >> 4)) % 3) == 0) ? (uint16_t)(base ^ 0xffff) : base;
    case SCN_MASK_NOISE:
        return (uint16_t)(base ^ 0x0001);
    }
    return base;
}

static void fillFrame(uint16_t* fb, Scenario scenario, int orientation, bool new_frame)
{
    const uint32_t w = sourceWidth(orientation);
    const uint32_t h = sourceHeight(orientation);
    for (uint32_t y = 0; y < h; y++)
    {
        uint16_t* p = fb + (w * y);
        for (uint32_t x = 0; x < w; x++)
        {
            *(p++) = scenarioPixel(scenario, x, y, w, h, new_frame);
        }
    }
}

static uint16_t canonicalSourcePixel(const uint16_t* fb, int orientation, int n)
{
    const int y = n / LX;
    const int x = n - (LX * y);
    switch (orientation)
    {
    case DiffBuffBase::PORTRAIT_240x320:
        return fb[n];
    case DiffBuffBase::LANDSCAPE_320x240:
        return fb[y + LY * (LX - 1 - x)];
    case DiffBuffBase::PORTRAIT_240x320_FLIPPED:
        return fb[NB_PIXELS - 1 - n];
    case DiffBuffBase::LANDSCAPE_320x240_FLIPPED:
        return fb[(LY - 1 - y) + LY * x];
    }
    return 0;
}

static uint64_t hashStep(uint64_t h, uint16_t v)
{
    h ^= v;
    h *= 1099511628211ull;
    return h;
}

static void printHash(uint64_t h)
{
    Serial.printf("%08lx%08lx", (uint32_t)(h >> 32), (uint32_t)h);
}

static uint64_t expectedHash(const uint16_t* new_fb, int orientation, uint16_t compare_mask)
{
    const uint16_t mask = compare_mask ? compare_mask : 0xffff;
    uint64_t h = 1469598103934665603ull;
    for (int n = 0; n < NB_PIXELS; n++)
    {
        h = hashStep(h, canonicalSourcePixel(new_fb, orientation, n) & mask);
    }
    return h;
}

static bool verifyRawDiff(DiffBuff& diff, const uint16_t* old_fb, const uint16_t* new_fb, int orientation, uint16_t compare_mask, uint64_t& out_hash, uint32_t& skip_errors)
{
    const uint16_t mask = compare_mask ? compare_mask : 0xffff;
    uint64_t h = 1469598103934665603ull;
    int pos = 0;
    skip_errors = 0;

    diff.initRaw();
    while (pos < NB_PIXELS)
    {
        int nbwrite = 0;
        int nbskip = 0;
        diff.readRaw(nbwrite, nbskip);

        if ((nbwrite == 0) && (nbskip > NB_PIXELS))
        {
            while (pos < NB_PIXELS)
            {
                const uint16_t oldp = old_fb[pos] & mask;
                const uint16_t newp = canonicalSourcePixel(new_fb, orientation, pos) & mask;
                if (oldp != newp) skip_errors++;
                h = hashStep(h, oldp);
                pos++;
            }
            break;
        }
        if (nbwrite > NB_PIXELS)
        {
            nbwrite = NB_PIXELS - pos;
            nbskip = 0;
        }
        if ((nbwrite < 0) || (nbskip < 0) || (pos + nbwrite > NB_PIXELS))
        {
            out_hash = h;
            return false;
        }

        for (int i = 0; i < nbwrite; i++, pos++)
        {
            h = hashStep(h, canonicalSourcePixel(new_fb, orientation, pos) & mask);
        }
        if (pos + nbskip > NB_PIXELS)
        {
            out_hash = h;
            return false;
        }
        for (int i = 0; i < nbskip; i++, pos++)
        {
            const uint16_t oldp = old_fb[pos] & mask;
            const uint16_t newp = canonicalSourcePixel(new_fb, orientation, pos) & mask;
            if (oldp != newp) skip_errors++;
            h = hashStep(h, oldp);
        }
    }

    out_hash = h;
    return (pos == NB_PIXELS);
}

static bool verifyMirror(const uint16_t* old_fb, const uint16_t* new_fb, int orientation, uint16_t compare_mask, uint64_t& out_hash)
{
    const uint16_t mask = compare_mask ? compare_mask : 0xffff;
    uint64_t h = 1469598103934665603ull;
    uint64_t expected = 1469598103934665603ull;
    for (int n = 0; n < NB_PIXELS; n++)
    {
        const uint16_t a = old_fb[n] & mask;
        const uint16_t b = canonicalSourcePixel(new_fb, orientation, n) & mask;
        h = hashStep(h, a);
        expected = hashStep(expected, b);
    }
    out_hash = h;
    return h == expected;
}

static void prepareCase(uint16_t* old_fb, uint16_t* new_fb, Scenario scenario, int orientation)
{
    fillFrame(new_fb, scenario, orientation, false);
    DiffBuffBase::copyfb(old_fb, new_fb, orientation);
    fillFrame(new_fb, scenario, orientation, true);
}

static bool checkCaseWithBuffers(DiffBuff& diff, uint16_t* old_fb, uint16_t* new_fb, Scenario scenario, int orientation, int gap, uint16_t compare_mask,
    uint64_t& raw_hash, uint64_t& mirror_hash, uint32_t& skip_errors)
{
    prepareCase(old_fb, new_fb, scenario, orientation);
    diff.computeDiff(old_fb, new_fb, orientation, gap, false, compare_mask);

    const uint64_t expected = expectedHash(new_fb, orientation, compare_mask);
    bool ok_raw = verifyRawDiff(diff, old_fb, new_fb, orientation, compare_mask, raw_hash, skip_errors);
    ok_raw = ok_raw && (raw_hash == expected) && (skip_errors == 0);

    prepareCase(old_fb, new_fb, scenario, orientation);
    diff.computeDiff(old_fb, new_fb, orientation, gap, true, compare_mask);
    const bool ok_mirror = verifyMirror(old_fb, new_fb, orientation, compare_mask, mirror_hash);

    return ok_raw && ok_mirror;
}

static bool checkCase(DiffBuff& diff, Scenario scenario, int orientation, int gap, uint16_t compare_mask, uint64_t& raw_hash, uint64_t& mirror_hash, uint32_t& skip_errors)
{
    return checkCaseWithBuffers(diff, fb_old, fb_new, scenario, orientation, gap, compare_mask, raw_hash, mirror_hash, skip_errors);
}

static BenchResult runBenchWithBuffers(DiffBuff& diff, uint16_t* old_fb, uint16_t* new_fb, Scenario scenario, int orientation, int gap, bool copy_new_over_old,
    uint16_t compare_mask)
{
    BenchResult r;
    r.total_us = 0;
    r.min_us = 0xffffffffu;
    r.max_us = 0;
    r.last_size = 0;
    r.overflow_count = 0;

    for (int i = 0; i < WARMUP; i++)
    {
        prepareCase(old_fb, new_fb, scenario, orientation);
        diff.computeDiff(old_fb, new_fb, orientation, gap, copy_new_over_old, compare_mask);
        sink += diff.size();
    }

    diff.statsReset();
    for (int i = 0; i < ITER; i++)
    {
        prepareCase(old_fb, new_fb, scenario, orientation);
        elapsedMicros em = 0;
        diff.computeDiff(old_fb, new_fb, orientation, gap, copy_new_over_old, compare_mask);
        const uint32_t dt = em;
        r.total_us += dt;
        if (dt < r.min_us) r.min_us = dt;
        if (dt > r.max_us) r.max_us = dt;
        r.last_size = diff.size();
        sink += r.last_size;
    }
    r.overflow_count = diff.statsNbOverflow();
    return r;
}

static BenchResult runBench(DiffBuff& diff, Scenario scenario, int orientation, int gap, bool copy_new_over_old, uint16_t compare_mask)
{
    return runBenchWithBuffers(diff, fb_old, fb_new, scenario, orientation, gap, copy_new_over_old, compare_mask);
}

static void printAvg(uint64_t total_us, int count)
{
    const uint32_t whole = (uint32_t)(total_us / count);
    const uint32_t frac = (uint32_t)(((total_us % count) * 1000ull) / count);
    Serial.printf("%lu.%03lu", whole, frac);
}

static void runOne(DiffBuff& diff, const char* diff_name, int diff_size, const ScenarioDef& scn, int orientation, int gap, bool copy_new_over_old)
{
    uint64_t raw_hash = 0;
    uint64_t mirror_hash = 0;
    uint32_t skip_errors = 0;
    const bool ok = checkCase(diff, scn.scenario, orientation, gap, scn.compare_mask, raw_hash, mirror_hash, skip_errors);
    const BenchResult r = runBench(diff, scn.scenario, orientation, gap, copy_new_over_old, scn.compare_mask);

    Serial.printf("diff=%s(%d),scn=%s,ori=%s,gap=%d,copy=%d,mask=%04x,avg=",
        diff_name, diff_size, scn.name, orientationName(orientation), gap, copy_new_over_old ? 1 : 0, scn.compare_mask);
    printAvg(r.total_us, ITER);
    Serial.printf("us,min=%lu,max=%lu,size=%d,of=%lu,check=%s,skiperr=%lu,raw=",
        r.min_us, r.max_us, r.last_size, r.overflow_count, ok ? "OK" : "FAIL", skip_errors);
    printHash(raw_hash);
    Serial.print(",mirror=");
    printHash(mirror_hash);
    Serial.println();
}

static void runOneWithBuffers(DiffBuff& diff, const char* diff_name, int diff_size, const ScenarioDef& scn, int orientation, int gap, bool copy_new_over_old,
    uint16_t* old_fb, uint16_t* new_fb)
{
    uint64_t raw_hash = 0;
    uint64_t mirror_hash = 0;
    uint32_t skip_errors = 0;
    const bool ok = checkCaseWithBuffers(diff, old_fb, new_fb, scn.scenario, orientation, gap, scn.compare_mask, raw_hash, mirror_hash, skip_errors);
    const BenchResult r = runBenchWithBuffers(diff, old_fb, new_fb, scn.scenario, orientation, gap, copy_new_over_old, scn.compare_mask);

    Serial.printf("diff=%s(%d),scn=%s,ori=%s,gap=%d,copy=%d,mask=%04x,avg=",
        diff_name, diff_size, scn.name, orientationName(orientation), gap, copy_new_over_old ? 1 : 0, scn.compare_mask);
    printAvg(r.total_us, ITER);
    Serial.printf("us,min=%lu,max=%lu,size=%d,of=%lu,check=%s,skiperr=%lu,raw=",
        r.min_us, r.max_us, r.last_size, r.overflow_count, ok ? "OK" : "FAIL", skip_errors);
    printHash(raw_hash);
    Serial.print(",mirror=");
    printHash(mirror_hash);
    Serial.println();
}

static void runSuiteForDiff(DiffBuff& diff, const char* diff_name, int diff_size, bool full_matrix)
{
    for (uint32_t s = 0; s < sizeof(scenarios) / sizeof(scenarios[0]); s++)
    {
        for (uint32_t o = 0; o < sizeof(orientations) / sizeof(orientations[0]); o++)
        {
            for (uint32_t g = 0; g < sizeof(gaps) / sizeof(gaps[0]); g++)
            {
                if (!full_matrix)
                {
                    const bool overflow_interesting = (scenarios[s].scenario == SCN_FRAGMENTED_8) || (scenarios[s].scenario == SCN_SPARSE_64) || (scenarios[s].scenario == SCN_BLOCKS);
                    if (!overflow_interesting || (gaps[g] != 4)) continue;
                }
                runOne(diff, diff_name, diff_size, scenarios[s], orientations[o], gaps[g], true);
                if (full_matrix && (gaps[g] == 4))
                {
                    runOne(diff, diff_name, diff_size, scenarios[s], orientations[o], gaps[g], false);
                }
            }
        }
    }
}

static void runUnalignedSuite()
{
    uint16_t* const old_unaligned = fb_old + 1;
    uint16_t* const new_unaligned = fb_new + 1;
    const ScenarioDef unaligned_scenarios[] =
    {
        { "none_unaligned",     SCN_NO_CHANGE,  0 },
        { "sparse64_unaligned", SCN_SPARSE_64,  0 },
        { "blocks_unaligned",   SCN_BLOCKS,     0 },
        { "all_unaligned",      SCN_ALL_CHANGE, 0 },
        { "mask_unaligned",     SCN_MASK_NOISE, 0xfffe },
    };
    for (uint32_t s = 0; s < sizeof(unaligned_scenarios) / sizeof(unaligned_scenarios[0]); s++)
    {
        runOneWithBuffers(diff_large, "unaligned", LARGE_DIFF_SIZE, unaligned_scenarios[s], DiffBuffBase::PORTRAIT_240x320, 4, true, old_unaligned, new_unaligned);
        runOneWithBuffers(diff_large, "unaligned", LARGE_DIFF_SIZE, unaligned_scenarios[s], DiffBuffBase::PORTRAIT_240x320, 4, false, old_unaligned, new_unaligned);
        runOneWithBuffers(diff_large, "unaligned", LARGE_DIFF_SIZE, unaligned_scenarios[s], DiffBuffBase::PORTRAIT_240x320_FLIPPED, 4, true, old_unaligned, new_unaligned);
        runOneWithBuffers(diff_large, "unaligned", LARGE_DIFF_SIZE, unaligned_scenarios[s], DiffBuffBase::PORTRAIT_240x320_FLIPPED, 4, false, old_unaligned, new_unaligned);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 4000) {}
    delay(500);

    Serial.println();
    Serial.println("ILI9341_T4 DiffBuff benchmark");
    Serial.printf("pixels=%d warmup=%d iter=%d large=%d small=%d\n", NB_PIXELS, WARMUP, ITER, LARGE_DIFF_SIZE, SMALL_DIFF_SIZE);
    Serial.println("CSV-ish fields: diff,scn,ori,gap,copy,mask,avg,min,max,size,of,check,skiperr,raw,mirror");
    Serial.println();

    Serial.println("=== LARGE DIFF BUFFER: broad matrix, normally no overflow except very fragmented cases ===");
    runSuiteForDiff(diff_large, "large", LARGE_DIFF_SIZE, true);

    Serial.println();
    Serial.println("=== SMALL DIFF BUFFER: selected overflow stress cases ===");
    runSuiteForDiff(diff_small, "small", SMALL_DIFF_SIZE, false);

    Serial.println();
    Serial.println("=== UNALIGNED ROT0/ROT180: uint16_t buffers shifted by one pixel ===");
    runUnalignedSuite();

    Serial.println();
    Serial.printf("sink=%lu\n", sink);
    Serial.println("Done.");
}

void loop()
{
}
