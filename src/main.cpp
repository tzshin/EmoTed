/* 
 * png to bmp with:
 * https://onlinepngtools.com/convert-png-to-bmp
 * 
 * bmp to 24 bit with:
 * https://online-converting.com/image/convert2bmp
 * 
 */

#include <Arduino.h>
#include <SPIFFS.h>

#include <vector>
#include <string>

#include <Servo.h>
#include <TFT_eSPI.h>

/*******************************/

#define DEBUG true

// Params
#define TED_WAVE_STEP     15
#define TED_WAVE_TRAVEL_S 0.6
#define TED_WAVE_MAX_DEG  95.0
#define TED_WAVE_MIN_DEG  30.0
#define TED_R_ARM_OFFSET  -10.0

#define IMAGE_X_POS  30
#define IMAGE_Y_POS  14

// PIN config
#define SERVO_L_PIN 16
#define SERVO_R_PIN 17

// #define TFT_MOSI    23
// #define TFT_SCLK    18
// #define TFT_CS      22
// #define TFT_DC      14
// #define TFT_RST     12
// #define TFT_LED     4

// Constants
#define S_TO_MILLIS   1000.0  // Convert second to millisecond

/*******************************/

// Debug mode implementation
#if DEBUG  // Turn on debug log
#define debug(x)           Serial.print(x)
#define debugln(x)         Serial.println(x)
#define debugf(x, args...) Serial.printf(x, ##args)
#else  // Trun off debug log
#define debug(x)
#define debugln(x)
#define debugf(x, args...)
#endif

/*******************************/

TFT_eSPI tft = TFT_eSPI();

/*******************************/

class WaveController {
public:
    WaveController(int step_n, float travel_s, float max, float min, float r_arm_offset) {
        this->servo_max = max;
        this->servo_min = min;
        this->r_arm_offset = r_arm_offset;
        
        // Make position to arm_angle map
        float step_deg = (max - min) / float(step_n - 1);
        for (int i = 0; i < step_n; i++) {
            this->position2deg.push_back(min + i * step_deg);
        }
        for (int i = 0; i < (step_n - 2); i++) {
            this->position2deg.push_back(max - (i+1) * step_deg);
        }

        this->position_t_interval = int(travel_s * S_TO_MILLIS / this->position2deg.size());

        this->servo.l.attach(SERVO_L_PIN);
        this->servo.r.attach(SERVO_R_PIN);
    }
    
    // Call tick() repeatedly to automate waving movement
    void tick() {
        if (this->is_waving &&
            millis() - this->waving_state.timestamp >= this->position_t_interval)
        {
            this->move_arm(int(this->position2deg.at(this->waving_state.position)));

            this->waving_state.timestamp = millis();
            this->waving_state.position += 1;
            if (this->waving_state.position == this->position2deg.size()) {
                this->waving_state.position = 0;
            }

            if (this->waving_end_timestamp != 0 &&
                millis() >= this->waving_end_timestamp) {
                this->waving_end_timestamp = 0;
                this->is_waving = false;
                this->waving_state.position = 0;
                this->reset_position();
            }
        }
    }

    // True to enable waving motion
    void wave(float time_s) {
        this->is_waving = true;
        this->waving_end_timestamp = millis() + (time_s * S_TO_MILLIS);
    }

    // Reset posture to resting position
    void reset_position() {
        this->move_arm(int(this->servo_min));
    }

    int get_position() {
        return int(this->position2deg.at(this->waving_state.position));
    }

private:
    struct {
        Servo l;
        Servo r;
    } servo;

    bool is_waving = false;
    struct {
        int position = 0;
        unsigned long timestamp = 0;
    } waving_state;

    unsigned long waving_end_timestamp = 0;

    float servo_max = 0.0;
    float servo_min = 0.0;
    float r_arm_offset = 0.0;
    std::vector<float> position2deg = {};
    int position_t_interval = 0;

    void move_arm(int angle) {
        this->servo.l.write(180 - angle);
        this->servo.r.write(angle + this->r_arm_offset);
    }
};

class DisplayController {
public:
    DisplayController(int img_x, int img_y) {
        this->img_x = img_x;
        this->img_y = img_y;
    }

    void tick() {
        if (this->is_displying &&
            !this->displaying_state.img_showing)
        {
            std::string fn = "/" + std::to_string(this->displaying_state.index) + ".bmp";
            drawBmp(fn.c_str(), this->img_x, this->img_y);

            this->displaying_state.img_showing = true;
        }

        if (this->displaying_state.img_showing &&
            millis() >= this->displaying_end_timestamp)
        {
            tft.fillScreen(TFT_BLACK);

            this->is_displying = false;
            this->displaying_state.img_showing = false;
        }
    }

    void display(int index, float time_s) {
        this->is_displying = true;
        this->displaying_state.index = index;
        this->displaying_end_timestamp = millis() + (time_s * S_TO_MILLIS);
    }
private:
    int16_t xpos = 0;
    int16_t ypos = 0;
    
    bool is_displying = false;
    struct {
        int index = 0;
        bool img_showing = false;
    } displaying_state;

    unsigned long displaying_end_timestamp = 0;

    int img_x = 0;
    int img_y = 0;

    // Bodmer's BMP image rendering function
    // https://github.com/Bodmer/TFT_eSPI/tree/master/examples/Generic/TFT_SPIFFS_BMP
    void drawBmp(const char *filename, int16_t x, int16_t y) {

        if ((x >= tft.width()) || (y >= tft.height())) return;

        fs::File bmpFS;

        // Open requested file on SD card
        bmpFS = SPIFFS.open(filename, "r");

        if (!bmpFS)
        {
            Serial.print("File not found");
            return;
        }

        uint32_t seekOffset;
        uint16_t w, h, row, col;
        uint8_t  r, g, b;

        uint32_t startTime = millis();

        if (read16(bmpFS) == 0x4D42)
        {
            read32(bmpFS);
            read32(bmpFS);
            seekOffset = read32(bmpFS);
            read32(bmpFS);
            w = read32(bmpFS);
            h = read32(bmpFS);

            if ((read16(bmpFS) == 1) && (read16(bmpFS) == 24) && (read32(bmpFS) == 0))
            {
                y += h - 1;

                bool oldSwapBytes = tft.getSwapBytes();
                tft.setSwapBytes(true);
                bmpFS.seek(seekOffset);

                uint16_t padding = (4 - ((w * 3) & 3)) & 3;
                uint8_t lineBuffer[w * 3 + padding];

                for (row = 0; row < h; row++) {
                    
                    bmpFS.read(lineBuffer, sizeof(lineBuffer));
                    uint8_t*  bptr = lineBuffer;
                    uint16_t* tptr = (uint16_t*)lineBuffer;
                    // Convert 24 to 16 bit colours
                    for (uint16_t col = 0; col < w; col++)
                    {
                    b = *bptr++;
                    g = *bptr++;
                    r = *bptr++;
                    *tptr++ = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
                    }

                    // Push the pixel row to screen, pushImage will crop the line if needed
                    // y is decremented as the BMP image is drawn bottom up
                    tft.pushImage(x, y--, w, 1, (uint16_t*)lineBuffer);
                }
                tft.setSwapBytes(oldSwapBytes);
                Serial.print("Loaded in "); Serial.print(millis() - startTime);
                Serial.println(" ms");
            }
            else Serial.println("BMP format not recognized.");
        }
        bmpFS.close();
    }

    // These read 16- and 32-bit types from the SD card file.
    // BMP data is stored little-endian, Arduino is little-endian too.
    // May need to reverse subscript order if porting elsewhere.
    uint16_t read16(fs::File &f) {
        uint16_t result;
        ((uint8_t *)&result)[0] = f.read(); // LSB
        ((uint8_t *)&result)[1] = f.read(); // MSB
        return result;
    }
    uint32_t read32(fs::File &f) {
        uint32_t result;
        ((uint8_t *)&result)[0] = f.read(); // LSB
        ((uint8_t *)&result)[1] = f.read();
        ((uint8_t *)&result)[2] = f.read();
        ((uint8_t *)&result)[3] = f.read(); // MSB
        return result;
    }
};

WaveController wave_controller(TED_WAVE_STEP, TED_WAVE_TRAVEL_S, TED_WAVE_MAX_DEG, TED_WAVE_MIN_DEG, TED_R_ARM_OFFSET);
DisplayController display_controller(IMAGE_X_POS, IMAGE_Y_POS);

/*******************************/

void setup() {
    Serial.begin(115200);
    debugln("Serial ok!");

    if (!SPIFFS.begin()) {
        while (1) yield();
    }
    debugln("SPIFFS ok!");

    tft.begin();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    debugln("TFT ok!");

    wave_controller.reset_position();
    debugln("Servo ok!");
}

/*******************************/

void loop() {
    wave_controller.tick();
    display_controller.tick();

    if (Serial.available()) {
        std::string input = std::string(Serial.readStringUntil('\n').c_str());
        float action_time = std::stof(input.substr(0, input.find(",")).c_str());
        int expression_index = std::stoi(input.substr(input.size()-1).c_str());
        
        wave_controller.wave(action_time);
        display_controller.display(expression_index, action_time);

        debugf("w: %.1f, d: %d\n", action_time, expression_index);
    }
}
