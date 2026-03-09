/*
    Industrial Sensor Data Simulator

    Reads current (A), voltage (V), and temperature (C) from simulated
    industrial sensors, formats each reading as a JSON payload, and sends
    it via HTTP POST to a cloud endpoint every 2 seconds.

    Usage:
        ./sensor_simulator           runs indefinitely until Ctrl+C
        ./sensor_simulator <count>   runs exactly <count> transmissions

    Secrets and endpoint config are loaded from .env file at startup.
*/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <csignal>
#include <chrono>
#include <thread>
#include <random>
#include <cmath>

// --- Optical Physics Engine (Cortem K13-253 Specification) ---
struct GlassSpecs {
    const char* n_mod;      // Model Number
    double phi_nominal;     // Ø Diameter (mm)
    double phi_tol_pos;     // Ø Tolerance +
    double phi_tol_neg;     // Ø Tolerance -
    double s_nominal;       // S Thickness (mm)
    double s_tol_pos;       // S Tolerance +
    double s_tol_neg;       // S Tolerance -

    double refractive_idx;  // 1.52 for tempered glass
    double alpha;           // Absorption coefficient per mm (~0.005)
};

struct OpticalDiagnostics {
    double s_max_mm;              // Calculated worst-case thickness
    double transmission_pct;      // Total light making it through
    double display_gain;          // Multiplier needed for 100% perceived brightness
    double surface_reflectance;   // Primary reflection at boundary
    double internal_glare_ratio;  // Secondary bounce causing halo
};

OpticalDiagnostics calculateOptics(const GlassSpecs& specs) {
    // Calculate worst-case optical depth (S nominal + max positive tolerance)
    double s_max = specs.s_nominal + specs.s_tol_pos;

    // 1. Boundary Reflection (Fresnel) at normal incidence
    double n1 = 1.0;                  // Air
    double n2 = specs.refractive_idx; // Glass
    double R  = std::pow((n1 - n2) / (n1 + n2), 2.0);

    // 2. Two-interface transmission (entry + exit surfaces)
    double transmittance_reflection = std::pow(1.0 - R, 2.0);

    // 3. Material Absorption (Beer-Lambert), primary ray traverses s_max once
    double transmittance_absorption = std::exp(-specs.alpha * s_max);

    // 4. Total Transmittance
    double total_transmittance = transmittance_reflection * transmittance_absorption;

    // 5. Internal Glare (ghost ray / secondary reflection)
    //
    //    Path of the ghost ray:
    //      Entry surface  → glass (1 pass, forward)
    //      Back surface   → reflects (R)
    //      Front surface  → reflects again (R)
    //      Exit surface   → exits glass
    //
    //    The ghost ray therefore:
    //      - Crosses both air-glass boundaries twice  → (1-R)²  factor
    //      - Reflects at back surface once            → R
    //      - Reflects at front surface once           → R
    //      - Traverses the thickness THREE times      → exp(-3αd)
    //
    //    internal_glare = (1-R)² · R² · exp(-3αd)
    double t_abs_triple  = std::exp(-specs.alpha * s_max * 3.0);
    double internal_glare = std::pow(1.0 - R, 2.0) * R * R * t_abs_triple;

    return {
        s_max,
        total_transmittance,
        1.0 / total_transmittance,
        R,
        internal_glare
    };
}

// Transmission settings
static const int POST_INTERVAL_S   = 2;
static const int MAX_RETRIES       = 3;
static const int RETRY_BASE_MS     = 250;
static const int WATCHDOG_INTERVAL = 10;

// File paths
static const char ENV_PATH[]      = ".env";
static const char LOG_PATH[]      = "/tmp/sensor_sim.log";
static const char WATCHDOG_PATH[] = "/tmp/sensor_sim.watchdog";

// Buffer sizes
static const int CFG_FIELD_LEN = 256;
static const int TIMESTAMP_LEN = 32;
static const int JSON_BUF_LEN  = 1024;
static const int LOG_LINE_LEN  = 256;
static const int ENV_LINE_LEN  = 512;

// Sensor operating ranges
static const double AMPS_MIN  =   0.5,  AMPS_MAX  =  20.0;
static const double VOLTS_MIN = 210.0,  VOLTS_MAX = 240.0;
static const double TEMP_MIN  =  20.0,  TEMP_MAX  =  85.0;


// Signal handling
static volatile sig_atomic_t g_running = 1;

void signalHandler(int sig) {
    (void)sig;
    g_running = 0;
}


// Logging
enum LogLevel { LOG_INFO = 0, LOG_WARN, LOG_ERROR };

static const char* LOG_LABELS[] = { "INFO ", "WARN ", "ERROR" };

void logWrite(LogLevel level, const char* message) {
    char timebuf[TIMESTAMP_LEN];
    char linebuf[LOG_LINE_LEN];

    std::time_t now = std::time(nullptr);
    std::strftime(timebuf, sizeof(timebuf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&now));
    std::snprintf(linebuf, sizeof(linebuf), "[%s] [%s] %s\n",
                  timebuf, LOG_LABELS[level], message);

    std::fputs(linebuf, stdout);

    FILE* fp = std::fopen(LOG_PATH, "a");
    if (fp) {
        std::fputs(linebuf, fp);
        std::fclose(fp);
    }
}


// Watchdog
void touchWatchdog() {
    FILE* fp = std::fopen(WATCHDOG_PATH, "w");
    if (!fp) {
        logWrite(LOG_WARN, "Watchdog file update failed");
        return;
    }
    char buf[TIMESTAMP_LEN];
    std::time_t now = std::time(nullptr);
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&now));
    std::fputs(buf, fp);
    std::fclose(fp);
}


// Config
struct Config {
    char serverUrl[CFG_FIELD_LEN];
    char authToken[CFG_FIELD_LEN];
    char deviceId [CFG_FIELD_LEN];
};

static char* strTrim(char* s) {
    while (*s == ' ' || *s == '\t') ++s;
    char* end = s + std::strlen(s);
    while (end > s && (*(end-1) == ' '  || *(end-1) == '\t' ||
                       *(end-1) == '\r' || *(end-1) == '\n')) --end;
    *end = '\0';
    return s;
}

bool loadConfig(const char* path, Config& cfg) {
    FILE* fp = std::fopen(path, "r");
    if (!fp) {
        logWrite(LOG_ERROR, "Cannot open .env file");
        return false;
    }

    std::memset(&cfg, 0, sizeof(cfg));

    char line[ENV_LINE_LEN];
    while (std::fgets(line, sizeof(line), fp)) {
        char* trimmed = strTrim(line);
        if (*trimmed == '#' || *trimmed == '\0') continue;

        char* eq = std::strchr(trimmed, '=');
        if (!eq) continue;

        *eq       = '\0';
        char* key = strTrim(trimmed);
        char* val = strTrim(eq + 1);

        if      (std::strcmp(key, "SERVER_URL") == 0) std::strncpy(cfg.serverUrl, val, CFG_FIELD_LEN - 1);
        else if (std::strcmp(key, "AUTH_TOKEN") == 0) std::strncpy(cfg.authToken, val, CFG_FIELD_LEN - 1);
        else if (std::strcmp(key, "DEVICE_ID")  == 0) std::strncpy(cfg.deviceId,  val, CFG_FIELD_LEN - 1);
    }

    std::fclose(fp);

    if (cfg.serverUrl[0] == '\0' || cfg.authToken[0] == '\0' || cfg.deviceId[0] == '\0') {
        logWrite(LOG_ERROR, "Missing required .env key(s): SERVER_URL, AUTH_TOKEN, DEVICE_ID");
        return false;
    }

    return true;
}


// Sensor reading
struct SensorReading {
    double    amps;
    double    volts;
    double    temperature;
    OpticalDiagnostics optics;
    char      timestamp[TIMESTAMP_LEN];
    long long sequence;
};

void readSensors(std::mt19937& rng, long long sequence, SensorReading& r) {
    auto uniform = [&](double lo, double hi) -> double {
        return std::uniform_real_distribution<double>(lo, hi)(rng);
    };

    // Bounded random walk to simulate gradual temperature drift
    static double tempDrift = 0.0;
    tempDrift += uniform(-0.3, 0.3);
    if      (tempDrift >  5.0) tempDrift =  5.0;
    else if (tempDrift < -5.0) tempDrift = -5.0;

    r.amps        = uniform(AMPS_MIN, AMPS_MAX);
    r.volts       = uniform(VOLTS_MIN, VOLTS_MAX);
    r.temperature = uniform(TEMP_MIN, TEMP_MAX) + tempDrift;
    if      (r.temperature > TEMP_MAX) r.temperature = TEMP_MAX;
    else if (r.temperature < TEMP_MIN) r.temperature = TEMP_MIN;
    r.sequence    = sequence;

    std::time_t now = std::time(nullptr);
    std::strftime(r.timestamp, sizeof(r.timestamp),
                  "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&now));

    // Calculate optics based on drawing: N° MOD K13-253
    GlassSpecs k13_253 = {
        "K13-253",
        100.0, 0.5, 0.5,   // Ø = 100 +0.5 / -0.5
        10.0,  0.2, 0.3,   // S = 10 +0.2 / -0.3
        1.52,  0.005       // n=1.52, alpha=0.005
    };
    r.optics = calculateOptics(k13_253);
}


// JSON serialisation
int buildJsonPayload(const SensorReading& r, const char* deviceId,
                     char* buf, int bufLen) {
    return std::snprintf(buf, static_cast<size_t>(bufLen),
        "{\n"
        "  \"device_id\"  : \"%s\",\n"
        "  \"sequence\"   : %lld,\n"
        "  \"timestamp\"  : \"%s\",\n"
        "  \"sensors\": {\n"
        "    \"current\":     { \"value\": %.3f, \"unit\": \"A\" },\n"
        "    \"voltage\":     { \"value\": %.3f, \"unit\": \"V\" },\n"
        "    \"temperature\": { \"value\": %.3f, \"unit\": \"C\" }\n"
        "  },\n"
        "  \"optics\": {\n"
        "    \"n_mod\": \"K13-253\",\n"
        "    \"diameter_phi\": { \"nominal\": 100.0, \"tol_pos\": 0.5, \"tol_neg\": 0.5 },\n"
        "    \"thickness_s\": { \"nominal\": 10.0, \"tol_pos\": 0.2, \"tol_neg\": 0.3 },\n"
        "    \"active_s_max_mm\": %.1f,\n"
        "    \"transmittance_pct\": %.4f,\n"
        "    \"display_gain\": %.4f,\n"
        "    \"surface_reflectance\": %.4f,\n"
        "    \"internal_glare_ratio\": %.6f\n"
        "  }\n"
        "}",
        deviceId, r.sequence, r.timestamp,
        r.amps, r.volts, r.temperature,
        r.optics.s_max_mm,
        r.optics.transmission_pct, r.optics.display_gain,
        r.optics.surface_reflectance, r.optics.internal_glare_ratio);
}


// HTTP POST
enum TransmitResult {
    TX_OK          = 0,
    TX_ERR_NETWORK = 1,
    TX_ERR_FATAL   = 2
};

/*
    libcurl integration reference:
        CURL* curl = curl_easy_init();
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, authHeader);
        curl_easy_setopt(curl, CURLOPT_URL,            serverUrl);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS,     payload);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER,     headers);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT,        5L);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 3L);
        CURLcode res = curl_easy_perform(curl);
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
*/

TransmitResult simulateHttpPost(const char* payload,
                                const char* serverUrl,
                                const char* authToken,
                                long long   sequence) {
    static std::mt19937 rng(42);

    int latencyMs = std::uniform_int_distribution<int>(10, 120)(rng);
    std::this_thread::sleep_for(std::chrono::milliseconds(latencyMs));

    // 1% fatal (401), 5% transient (503), 94% success (200)
    double         roll   = std::uniform_real_distribution<double>(0.0, 1.0)(rng);
    TransmitResult result;
    const char*    status;

    if (roll < 0.01) {
        result = TX_ERR_FATAL;
        status = "401 Unauthorized";
    } else if (roll < 0.06) {
        result = TX_ERR_NETWORK;
        status = "503 Service Unavailable";
    } else {
        result = TX_OK;
        status = "200 OK";
    }

    std::printf("POST #%lld  url=%s  latency=%dms  status=%s\n",
                sequence, serverUrl, latencyMs, status);
    std::printf("auth: %s\n", authToken);
    std::printf("%s\n\n", payload);

    return result;
}


// Main
int main(int argc, char* argv[]) {
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Parse optional transmission count argument
    long long maxTransmissions = 0;
    if (argc >= 2) {
        char* endptr = nullptr;
        long long n  = std::strtoll(argv[1], &endptr, 10);
        if (*endptr != '\0' || n <= 0) {
            std::fprintf(stderr, "Error: expected a positive integer, got '%s'\n", argv[1]);
            return 1;
        }
        maxTransmissions = n;
    }

    // Load config from .env
    Config cfg;
    if (!loadConfig(ENV_PATH, cfg)) return 1;

    // Print startup info
    char modeLabel[64];
    if (maxTransmissions > 0)
        std::snprintf(modeLabel, sizeof(modeLabel), "%lld transmissions", maxTransmissions);
    else
        std::strncpy(modeLabel, "indefinite", sizeof(modeLabel) - 1);

    std::printf("Sensor Simulator\n");
    std::printf("  device   : %s\n", cfg.deviceId);
    std::printf("  target   : %s\n", cfg.serverUrl);
    std::printf("  mode     : %s\n", modeLabel);
    std::printf("  interval : %ds\n", POST_INTERVAL_S);
    std::printf("  optics   : Spec N° MOD. K13-253 Active\n\n");

    logWrite(LOG_INFO, "Simulator started");

    std::mt19937 rng(std::random_device{}());

    // Pre-allocate buffers outside the loop
    SensorReading reading;
    char          jsonBuf[JSON_BUF_LEN];
    char          logMsg [LOG_LINE_LEN];

    long long sequence  = 1;
    int       successes = 0;
    int       failures  = 0;

    while (g_running && (maxTransmissions == 0 || sequence <= maxTransmissions)) {
        auto cycleStart = std::chrono::steady_clock::now();

        // Stage 1: read sensors
        readSensors(rng, sequence, reading);

        // Stage 2: build JSON
        int written = buildJsonPayload(reading, cfg.deviceId, jsonBuf, JSON_BUF_LEN);
        if (written < 0 || written >= JSON_BUF_LEN) {
            logWrite(LOG_ERROR, "JSON buffer overflow");
        }

        // Stage 3: transmit with exponential backoff retry
        TransmitResult result    = TX_ERR_NETWORK;
        int            attempt   = 0;
        int            backoffMs = RETRY_BASE_MS;

        while (attempt < MAX_RETRIES && result == TX_ERR_NETWORK) {
            if (attempt > 0) {
                std::snprintf(logMsg, sizeof(logMsg),
                              "Retry %d/%d seq=%lld backoff=%dms",
                              attempt, MAX_RETRIES, sequence, backoffMs);
                logWrite(LOG_WARN, logMsg);
                std::this_thread::sleep_for(std::chrono::milliseconds(backoffMs));
                backoffMs *= 2;
            }
            result = simulateHttpPost(jsonBuf, cfg.serverUrl, cfg.authToken, sequence);
            ++attempt;
        }

        // Stage 4: record outcome
        if (result == TX_OK) {
            ++successes;
        } else {
            ++failures;
            std::snprintf(logMsg, sizeof(logMsg),
                          "%s seq=%lld attempts=%d",
                          (result == TX_ERR_FATAL) ? "Fatal error" : "Failed after retries",
                          sequence, attempt);
            logWrite((result == TX_ERR_FATAL) ? LOG_ERROR : LOG_WARN, logMsg);
        }

        std::printf("stats  sent=%lld  ok=%d  failed=%d  rate=%.1f%%\n\n",
                    sequence, successes, failures,
                    100.0 * successes / static_cast<double>(sequence));

        // Stage 5: update watchdog periodically
        if (sequence % WATCHDOG_INTERVAL == 0) {
            touchWatchdog();
        }

        ++sequence;

        // Stage 6: sleep for the remainder of the interval
        auto elapsed  = std::chrono::steady_clock::now() - cycleStart;
        auto sleepFor = std::chrono::seconds(POST_INTERVAL_S) - elapsed;
        if (sleepFor > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(sleepFor);
        }
    }

    std::printf("done  total=%lld  ok=%d  failed=%d\n", sequence - 1, successes, failures);
    logWrite(LOG_INFO, "Simulator stopped");
    return 0;
}
