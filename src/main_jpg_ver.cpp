#include <Arduino.h>

#include <vector>
#include <string>
#include <SPIFFS.h>

#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <JPEGDecoder.h>

/*******************************/

#define DEBUG true

// Params
#define TED_WAVE_STEP     30
#define TED_WAVE_TRAVEL_S 2
#define TED_WAVE_MAX_DEG  40.0
#define TED_WAVE_MIN_DEG  0.0

// PIN config
#define SERVO_L_PIN 16
#define SERVO_R_PIN 17
#define TFT_MOSI    23
#define TFT_SCLK    18
#define TFT_CS      22
#define TFT_DC      14
#define TFT_RST     12

// Constants
#define S_TO_MILLIS   1000.0  // Convert second to millisecond

#define minimum(a,b) (((a)<(b))?(a):(b))

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

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

/*******************************/

void jpegRender(int xpos, int ypos) {
    // retrieve infomration about the image
    uint16_t *pImg;
    uint16_t mcu_w = JpegDec.MCUWidth;
    uint16_t mcu_h = JpegDec.MCUHeight;
    uint32_t max_x = JpegDec.width;
    uint32_t max_y = JpegDec.height;

    // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
    // Typically these MCUs are 16x16 pixel blocks
    // Determine the width and height of the right and bottom edge image blocks
    uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
    uint32_t min_h = minimum(mcu_h, max_y % mcu_h);

    // save the current image block size
    uint32_t win_w = mcu_w;
    uint32_t win_h = mcu_h;

    // record the current time so we can measure how long it takes to draw an image
    uint32_t drawTime = millis();

    // save the coordinate of the right and bottom edges to assist image cropping to the screen size
    max_x += xpos;
    max_y += ypos;

    // read each MCU block until there are no more
    while ( JpegDec.read()) {
        // save a pointer to the image block
        pImg = JpegDec.pImage;

        // calculate where the image block should be drawn on the screen
        int mcu_x = JpegDec.MCUx * mcu_w + xpos;
        int mcu_y = JpegDec.MCUy * mcu_h + ypos;

        // check if the image block size needs to be changed for the right edge
        if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
        else win_w = min_w;

        // check if the image block size needs to be changed for the bottom edge
        if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
        else win_h = min_h;

        // copy pixels into a contiguous block
        if (win_w != mcu_w) {
            for (int h = 1; h < win_h-1; h++) {
                memcpy(pImg + h * win_w, pImg + (h + 1) * mcu_w, win_w << 1);
            }
        }

        // draw image MCU block only if it will fit on the screen
        if ( ( mcu_x + win_w) <= tft.width() && ( mcu_y + win_h) <= tft.height()) {
            tft.drawRGBBitmap(mcu_x, mcu_y, pImg, win_w, win_h);
        }

        // Stop drawing blocks if the bottom of the screen has been reached,
        // the abort function will close the file
        else if ( ( mcu_y + win_h) >= tft.height()) JpegDec.abort();
    }

    // calculate how long it took to draw the image
    drawTime = millis() - drawTime;

    // print the results to the serial port
    Serial.print  ("Total render time was    : "); Serial.print(drawTime); Serial.println(" ms");
    Serial.println("=====================================");
}

void drawFSJpeg(const char *filename, int xpos, int ypos) {
    Serial.println("=====================================");
    Serial.print("Drawing file: "); Serial.println(filename);
    Serial.println("=====================================");

    // Open the file (the Jpeg decoder library will close it)
    fs::File jpgFile = SPIFFS.open( filename, "r");    // File handle reference for SPIFFS
    //  File jpgFile = SD.open( filename, FILE_READ);  // or, file handle reference for SD library
    
    if ( !jpgFile ) {
        Serial.print("ERROR: File \""); Serial.print(filename); Serial.println ("\" not found!");
        return;
    }

    // To initialise the decoder and provide the file, we can use one of the three following methods:
    //boolean decoded = JpegDec.decodeFsFile(jpgFile); // We can pass the SPIFFS file handle to the decoder,
    //boolean decoded = JpegDec.decodeSdFile(jpgFile); // or we can pass the SD file handle to the decoder,
    boolean decoded = JpegDec.decodeFsFile(filename);  // or we can pass the filename (leading / distinguishes SPIFFS files)
                                                        // The filename can be a String or character array
    if (decoded) {
        // print information about the image to the serial port
        // jpegInfo();

        // render the image onto the screen at given coordinates
        jpegRender(xpos, ypos);
    }
    else {
        Serial.println("Jpeg file format not supported!");
    }
}

/*******************************/

class WaveController {
public:
    WaveController(int step_n, float travel_s, float max, float min) {
        this->servo_max = max;
        this->servo_min = min;
        
        // Make position_map
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
            this->servo.l.write(int(this->position2deg.at(this->waving_state.position)));
            this->servo.r.write(int(this->position2deg.at(this->waving_state.position)));

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
        this->servo.l.write(int(this->servo_min));
        this->servo.r.write(int(this->servo_min));
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
    std::vector<float> position2deg = {};
    int position_t_interval = 0;
};

class DisplayController {
public:
    DisplayController(int a) {

    }

    void tick() {

    }

    void display(int index, float time_s) {

    }
private:

};

WaveController wave_controller(TED_WAVE_STEP, TED_WAVE_TRAVEL_S, TED_WAVE_MAX_DEG, TED_WAVE_MIN_DEG);
DisplayController display_controller(1);

/*******************************/

void setup() {
    Serial.begin(115200);
    debugln("Serial ok!");

    SPIFFS.begin(true);
    debugln("SPIFFS ok!");

    tft.initR(INITR_BLACKTAB);
    tft.fillScreen(ST7735_BLACK);
    debugln("TFT ok!");
}

/*******************************/

void loop() {
    wave_controller.tick();

    if (Serial.available()) {
        std::string input = std::string(Serial.readStringUntil('\n').c_str());
        float action_time = std::stof(input.substr(0, input.find(",")).c_str());
        int expression_index = std::stoi(input.substr(input.size()-1).c_str());

        wave_controller.wave(action_time);
        // display_controller.display(expression_index, action_time);

        debugf("w: %.1f, d: %d\n", action_time, expression_index);
    }
}
