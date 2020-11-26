/********************************************************************
* 
* ILI9341_T4 library example: displaying moving sprites... 
*
********************************************************************/

#include "ILI9341Driver.h"



/********************************************************************
* code for drawing colored balls.
********************************************************************/

// drawing size in landscape mode
#define LX  320
#define LY  240


/** fill a framebuffer with a given color*/
void clear(uint16_t* fb, uint16_t color = 0)
    {
    for (int i = 0; i < LX*LY; i++) fb[i] = color; 
    }

/** draw a disk centered at (x,y) with radius r and color col on the framebuffer fb */
void drawDisk(uint16_t * fb, double x, double y, double r, uint16_t col)
    {
    int xmin = (int)(x - r);
    int xmax = (int)(x + r);
    int ymin = (int)(y - r);
    int ymax = (int)(y + r);
    if (xmin < 0) xmin = 0;
    if (xmax >= LX) xmax = LX - 1;
    if (ymin < 0) ymin = 0;
    if (ymax >= LY) ymax = LY - 1;
    const double r2 = r * r;
    for (int j = ymin; j <= ymax; j++)
        {
        double dy2 = (y - j)*(y -j);
        for (int i = xmin; i <= xmax; i++)
            {
            const double dx2 = (x - i) * (x - i);
            if (dx2 + dy2 <= r2) fb[i + (j * LX)] = col;
            }
        }
    }

/** return a uniform in [0,1) */
double unif()
    {
    return random(2147483647) / 2147483647.0;
    }

/** a bouncing ball */
struct Ball
    {
    double x, y, dirx, diry, r; // position, direction, radius. 
    uint16_t color;

    Ball()
        { 
        r = unif() * 25; // random radius
        x = r; // start at the corner
        y = r; //
        dirx = unif() * 5; // direction and speed are random...
        diry = unif() * 5; // ...but not isotropic !
        color = random(65536); // random color
        }

    void move()
        {
        // move
        x += dirx;
        y += diry;
        // and bounce against border
        if (x - r < 0) { x = r;  dirx = -dirx; }
        if (y - r < 0) { y = r;  diry = -diry; }
        if (x > LX - r) { x = LX - r;  dirx = -dirx; }
        if (y > LY - r) { y = LY - r;  diry = -diry; }
        }

    void draw(uint16_t * fb)
        {
        drawDisk(fb, x, y, r, color);
        }
    };





/********************************************************************
* Main display code. 
* 
* With 30MHz SPI, the theoretical maximum framerate when doing full
* framebuffer redraw is 24 FPS. 
* 
* Here, we get around 70 FPS (without vsync) and a stable 50 FPS with 
* vsync (guaranteed screen tearing free!).
********************************************************************/


// 30MHz SPI. We can do much better with short wires
#define SPI_SPEED		30000000


// set the pins here (here I use SPI1)
// ***  Recall that DC must be on a valid cs pin !!! ***
#define PIN_SCK			27
#define PIN_MISO		1
#define PIN_MOSI		26
#define PIN_DC			0
#define PIN_RESET		29
#define PIN_CS			30
#define PIN_BACKLIGHT   28  // 255 if not connected to MCU. 
#define PIN_TOUCH_IRQ	32  // 255 if not used (or not on the same spi bus)
#define PIN_TOUCH_CS	31  // 255 if not used (or not on the same spi bus)


// the screen driver object
ILI9341_T4::ILI9341Driver tft(PIN_CS, PIN_DC, PIN_SCK, PIN_MOSI, PIN_MISO, PIN_RESET, PIN_TOUCH_CS, PIN_TOUCH_IRQ);


// 2 diff buffers with about 6K memory each
ILI9341_T4::DiffBuffStatic<6000> diff1;
ILI9341_T4::DiffBuffStatic<6000> diff2;


// framebuffers
uint16_t internal_fb[LX*LY];     // used for buffering
uint16_t fb[LX*LY];              // main framebuffer we draw onto.


// our 99 luftballons
Ball balls[99];


void setup()
    {
    Serial.begin(9600); 

    if (!tft.begin(SPI_SPEED))
        Serial.println("Initialization error...");

    // configuration for double buffering with two diff buffers
    tft.setRotation(1);                 // landscape mode 320x240 
    tft.setFramebuffers(internal_fb);   // set the internal framebuffer
    tft.setDiffBuffers(&diff1, &diff2); // set the diff buffers        
    tft.setDiffGap(5);                  // use a small gap for the diff buffers 
    tft.setDiffSplit(6);                // standard value
    tft.setRefreshRate(100);            // 100 hz refresh

    // vsync_spacing = 2 means we want 100/2 = 50 Hz fixed framerate with vsync enabled.
	// Try also setting this to 0 to find the maximum framerate (without vsync).
    tft.setVsyncSpacing(2); 

    if (PIN_BACKLIGHT != 255)
        { // make sure backlight is on
        pinMode(PIN_BACKLIGHT, OUTPUT);
        digitalWrite(PIN_BACKLIGHT, HIGH);        
        }
    }



int nbf = 0; // count the number of frames drawn. 

void loop()
    {
        clear(fb, 0); // erase the framebuffer, black background. 

        for (auto& b : balls)
        { // move and then draw all the balls onto the framebuffer
            b.move();
            b.draw(fb);
        }

        tft.update(fb); // push the framebuffer to be displayed

        if (++nbf % 2000 == 500)
        { // prints stats every 2000 frames. 
            Serial.println("************************************************************");
            tft.printStats();
            diff1.printStats();
            diff2.printStats();
        }
    }

