/********************************************************************
* 
* ILI9341_T4 library example. vsync and screen tearing demo. 
*
********************************************************************/

#include "Arduino.h"
#include "ILI9341Driver.h"


// drawing size in landscape mode
#define LX  320
#define LY  240


/** fill a framebuffer with a given color */
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



/********************************************************************
* This example demonstrates the effect of vsync on screen tearing.
* At each frame, a disk is drawn, alternatingly in red and blue. 
* 
* When vsync is activated and the framerate becomes high enough, the 
* disk appear to be solid violet ! When vsync is disabled, screen 
* tearing becomes apparent... 
********************************************************************/


// 30MHz SPI. We can do much better with short wires
#define SPI_SPEED		30000000


// set the pins here (I use SPI1)
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
// (in this simple case, only 1K memory buffer would be enough). 
ILI9341_T4::DiffBuffStatic<6000> diff1;
ILI9341_T4::DiffBuffStatic<6000> diff2;


// framebuffers
uint16_t internal_fb[LX*LY];     // used for buffering
uint16_t fb[LX*LY];              // main framebuffer we draw onto.


void setup()
    {
    Serial.begin(9600); 

    if (!tft.begin(SPI_SPEED))
        Serial.println("Initialization error...");

    // configuration for double buffering with two diff buffers
    tft.setRotation(1);                 // landscape mode 320x240 
    tft.setFramebuffers(internal_fb);   // set the internal framebuffer
    tft.setDiffBuffers(&diff1, &diff2); // set the diff buffers        
  
    if (PIN_BACKLIGHT != 255)
        { // make sure backlight is on
        pinMode(PIN_BACKLIGHT, OUTPUT);
        digitalWrite(PIN_BACKLIGHT, HIGH);        
        }

    tft.setRefreshRate(40); // start with a screen refresh rate around 40hz
    tft.setVsyncSpacing(8); // and a frame rate around 40/8 = 5Hz (slow !!!)
    }



const uint16_t BLUE = 31;           // colors
const uint16_t RED = (31 << 11);    // there is no violet here :-)

int n = 0; // counter
int a = 0; // angle

void loop()
    {
    clear(fb, 65535); // draw a white background 

    int x = 160 + 80 * cos(a * 0.03);  // the center of the disk 
    int y = 120 + 40 * sin(a * 0.03);  // follows an ellipse.
    drawDisk(fb, x, y, 60, ((++a & 1) ? RED : BLUE)); // alternate blue and red color at each frame. 

    tft.update(fb); // push the framebuffer to be displayed

    adjustFramerate(); // change the framerate / operation mode.  
    }


void adjustFramerate()
    {
    switch (n++)
        {
    default:    
        return;
    case 0:     
        break; // we start around 5Hz
    case 15:
        tft.setVsyncSpacing(4);  // around 10hz (framerate = refreshrate/4)
        break;
    case 45:
        tft.setVsyncSpacing(2);  // around 20 Hz (framerate = refreshrate/2)
        break;
    case 100:
        tft.setVsyncSpacing(1);  // around 40 Hz (framerate = refreshrate)
        break;
    case 200:
        tft.setRefreshRate(60); // around 60 Hz 
        break;
    case 350:
        tft.setRefreshRate(90); // around 90 Hz
        break;
    case 550:
        tft.setVsyncSpacing(0);  // disable vsync => screen tearing !
        break;
    case 800:
        tft.setVsyncSpacing(1);  // vsync back on => no more tearing :-)
        n = 351; // loop. 
        }
    if (tft.getVsyncSpacing() > 0)
        Serial.printf("- Alternating red / blue circles at %u fps. VSYNC ENABLED\n", (int)round(tft.getRefreshRate() / tft.getVsyncSpacing()));
    else
        Serial.printf("- Alternating red / blue circles at maximum fps. VSYNC DISABLED)\n");
    }

