//
// Created by Avaneesh on 25-07-2026.
//

//
// Created by Avaneesh on 25-07-2026.
//

#include "esp_camera.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "img_converters.h"
#include <WiFi.h>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <Arduino.h>

// --- WIFI CREDENTIALS ---
const char* ssid = "Netgear_Test";
const char* password = "coep@123";

// --- CAMERA PIN DEFINITIONS (AI-THINKER ESP32-CAM) ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// --- SETTINGS ---
#define CAM_WIDTH  320 // QVGA
#define CAM_HEIGHT 240
#define ROI_START_Y (CAM_HEIGHT / 2) // Only process bottom half
static const char* TAG = "cam_main";

// --- RANSAC STRUCTURES ---
struct Point2D { int x, y; };
struct LineModel { double m; double c; };

// --- HELPER FUNCTIONS ---

// 1. Initialize Camera
bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE; // IMPORTANT: Grayscale for processing
    config.frame_size = FRAMESIZE_QVGA;       // 320x240
    config.jpeg_quality = 12;
    config.fb_count = 2;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return false;
    }
    return true;
}

// 2. Simple Edge Detection (Replaces Canny)
std::vector<Point2D> detectEdges(camera_fb_t *fb) {
    std::vector<Point2D> edges;
    int threshold = 80; // Gradient threshold

    for (int y = ROI_START_Y; y < CAM_HEIGHT - 1; y++) {
        for (int x = 1; x < CAM_WIDTH - 1; x++) {
            int idx = (y * CAM_WIDTH) + x;
            int px_left  = fb->buf[idx - 1];
            int px_right = fb->buf[idx + 1];
            int dx = abs(px_right - px_left);

            int px_up   = fb->buf[idx - CAM_WIDTH];
            int px_down = fb->buf[idx + CAM_WIDTH];
            int dy = abs(px_down - px_up);

            if ((dx + dy) > threshold) {
                edges.push_back({x, y});
            }
        }
    }
    return edges;
}

// 3. RANSAC Logic
LineModel runRANSAC(const std::vector<Point2D>& points) {
    LineModel bestLine = {0, 0};
    if (points.size() < 2) return bestLine;

    int maxInliers = 0;
    int iterations = 30;
    double distThresh = 4.0;

    for (int i = 0; i < iterations; i++) {
        int idx1 = rand() % points.size();
        int idx2 = rand() % points.size();
        if (idx1 == idx2) continue;

        Point2D p1 = points[idx1];
        Point2D p2 = points[idx2];

        double m, c;
        if (p2.x == p1.x) { m = 1000; c = p1.x; }
        else {
            m = (double)(p2.y - p1.y) / (p2.x - p1.x);
            c = p1.y - m * p1.x;
        }

        int currentInliers = 0;
        for (const auto& p : points) {
            double d = std::abs(p.y - m * p.x - c) / std::sqrt(m * m + 1);
            if (d < distThresh) currentInliers++;
        }

        if (currentInliers > maxInliers) {
            maxInliers = currentInliers;
            bestLine = {m, c};
        }
    }
    return bestLine;
}

// 4. Drawing Helper
// Modifies the grayscale buffer directly to draw the line
void drawLine(uint8_t* buf, LineModel line, int width, int height) {
    // If slope is 0 (horizontal or invalid), skip
    if (std::abs(line.m) < 0.01) return;

    // Iterate through Y in the ROI and calculate X
    for (int y = ROI_START_Y; y < height; y++) {
        // x = (y - c) / m
        int x = (int)((y - line.c) / line.m);

        // Draw if point is within bounds
        if (x >= 0 && x < width) {
            // Draw a slightly thicker line (3px wide) for visibility
            buf[y * width + x] = 255;       // Center
            if(x > 0) buf[y * width + (x-1)] = 255; // Left
            if(x < width-1) buf[y * width + (x+1)] = 255; // Right
        }
    }
}

// --- WEB SERVER STREAM HANDLER ---
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    // Stream Loop
    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        // --- PROCESSING START ---

        // 1. Detect Edges
        std::vector<Point2D> allEdges = detectEdges(fb);

        // 2. Split Points
        std::vector<Point2D> leftPoints;
        std::vector<Point2D> rightPoints;
        for (auto p : allEdges) {
            if (p.x < CAM_WIDTH / 2) leftPoints.push_back(p);
            else rightPoints.push_back(p);
        }

        // 3. RANSAC
        LineModel leftLane = runRANSAC(leftPoints);
        LineModel rightLane = runRANSAC(rightPoints);

        // 4. Draw Lines on Frame Buffer (Visual Feedback)
        drawLine(fb->buf, leftLane, CAM_WIDTH, CAM_HEIGHT);
        drawLine(fb->buf, rightLane, CAM_WIDTH, CAM_HEIGHT);

        // --- PROCESSING END ---

        // Convert GrayScale FB to JPEG for Browser Streaming
        // (This is necessary because browsers can't easily display raw grayscale bytes)
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;

        if (!jpeg_converted) {
            ESP_LOGE(TAG, "JPEG compression failed");
            res = ESP_FAIL;
        } else {
            // Send HTTP Chunk
            if (res == ESP_OK) {
                size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
                res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
                if (res == ESP_OK) {
                    res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
                }
                if (res == ESP_OK) {
                    res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
                }
            }
        }

        if (_jpg_buf) {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }

        if (res != ESP_OK) break;
    }
    return res;
}

void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    httpd_handle_t stream_httpd = NULL;
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}

void setup() {
    Serial.begin(115200);

    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("WiFi connected: ");
    Serial.println(WiFi.localIP());

    // Init Camera
    if (!initCamera()) {
        Serial.println("Camera Init Failed");
        while(1);
    }

    // Start Server
    startCameraServer();
    Serial.print("Stream ready at: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/stream");
}

void loop() {
    // Empty loop - all logic is inside the stream_handler
    delay(1000);
}
// --- END OF CODE ---