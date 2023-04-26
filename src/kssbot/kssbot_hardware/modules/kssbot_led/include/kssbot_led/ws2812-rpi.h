/*
###############################################################################
#                                                                             #
# WS2812-RPi                                                                  #
# ==========                                                                  #
# A C++ library for driving WS2812 RGB LED's (known as 'NeoPixels' by         #
#     Adafruit) directly from a Raspberry Pi with accompanying Python wrapper #
# Copyright (C) 2014 Rob Kent                                                 #
#                                                                             #
# This program is free software: you can redistribute it and/or modify        #
# it under the terms of the GNU General Public License as published by        #
# the Free Software Foundation, either version 3 of the License, or           #
# (at your option) any later version.                                         #
#                                                                             #
# This program is distributed in the hope that it will be useful,             #
# but WITHOUT ANY WARRANTY; without even the implied warranty of              #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               #
# GNU General Public License for more details.                                #
#                                                                             #
# You should have received a copy of the GNU General Public License           #
# along with this program.  If not, see <http://www.gnu.org/licenses/>.       #
#                                                                             #
###############################################################################
*/
#ifndef WS2812_RPI_H
#define WS2812_RPI_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <dirent.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <signal.h>

#include <vector>
#include "ws2812-rpi-defines.h"

/*
 +-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+
 | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
 |   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5v      |     |     |
 |   3 |   9 |   SCL.1 | ALT0 | 1 |  5 || 6  |   |      | 0v      |     |     |
 |   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 1 | IN   | TxD     | 15  | 14  |
 |     |     |      0v |      |   |  9 || 10 | 1 | IN   | RxD     | 16  | 15  |
 |  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
 |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
 |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
 |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
 |  10 |  12 |    MOSI | ALT0 | 0 | 19 || 20 |   |      | 0v      |     |     |
 |   9 |  13 |    MISO | ALT0 | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
 |  11 |  14 |    SCLK | ALT0 | 0 | 23 || 24 | 1 | OUT  | CE0     | 10  | 8   |
 |     |     |      0v |      |   | 25 || 26 | 1 | OUT  | CE1     | 11  | 7   |
 |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
 |   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
 |   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
 |  13 |  23 | GPIO.23 |   IN | 0 | 33 || 34 |   |      | 0v      |     |     |
 |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
 |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
 |     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 +-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+
*/

class NeoPixel {
public:
    NeoPixel(unsigned int n);
    ~NeoPixel();

    void begin();
    void show();

    unsigned long millis(void){
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (ts.tv_sec*1000+ts.tv_nsec/1000000L);
    }

    unsigned char setPixelColor(unsigned int n, unsigned char r, unsigned char g, unsigned char b);
    unsigned char setPixelColor(unsigned int n, Color_t c);
    bool setBrightness(float b);

    //Color_t* getPixels();
    std::vector<Color_t> getPixels();
    float getBrightness();
    Color_t getPixelColor(unsigned int n);

    unsigned int numPixels();

    void clear();

    static Color_t wheel(uint8_t wheelPos);
    void colorWipe(Color_t c, uint8_t wait);
    void rainbow(uint8_t wait);
    void rainbowCycle(uint8_t wait);
    void theaterChase(Color_t c, uint8_t wait);
    void theaterChaseRainbow(uint8_t wait);

    static long map(long x, long in_min, long in_max, long out_min, long out_max);
    static Color_t gradientColor(std::vector<Color_t>& scheme, int range, int gradRange, int i);
    void gradient(std::vector<Color_t>& scheme, int repeat=1, int speedMS=1000);
    void bars(std::vector<Color_t>& scheme, int width=1, int speedMS=1000);

    void effectsDemo();

private:
    static void printBinary(unsigned int i, unsigned int bits);
    static unsigned int reverseWord(unsigned int word);

    void terminate(int dummy);
    void fatal(char *fmt, ...);

    unsigned int mem_virt_to_phys(void *virt);
    unsigned int mem_phys_to_virt(uint32_t phys);
    void* map_peripheral(uint32_t base, uint32_t len);

    void clearPWMBuffer();
    void clearLEDBuffer();

    static Color_t RGB2Color(unsigned char r, unsigned char g, unsigned char b);
    static Color_t Color(unsigned char r, unsigned char g, unsigned char b);

    void setPWMBit(unsigned int bitPos, unsigned char bit);
    unsigned char getPWMBit(unsigned int bitPos);

    void initHardware();
    void startTransfer();

    unsigned int numLEDs;
    std::vector<Color_t> LEDBuffer;
    float brightness;
    unsigned int PWMWaveform[NUM_DATA_WORDS];

    static struct control_data_s *ctl;
    page_map_t *page_map;
    static uint8_t *virtbase;

    static volatile unsigned int *pwm_reg;
    static volatile unsigned int *clk_reg;
    static volatile unsigned int *dma_reg;
    static volatile unsigned int *gpio_reg;
};

#endif
