#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/devicetree.h>
#include "text.h"
#include <zephyr/drivers/display.h>
#include <string.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>            // for abs


#define TRIG_NODE DT_ALIAS(trig)
#define ECHO_NODE DT_ALIAS(echo)

static const struct gpio_dt_spec trig = GPIO_DT_SPEC_GET(TRIG_NODE, gpios);
static const struct gpio_dt_spec echo = GPIO_DT_SPEC_GET(ECHO_NODE, gpios);

#define DISPLAY_WIDTH      256
#define DISPLAY_HEIGHT     120
#define DISPLAY_PITCH      DISPLAY_WIDTH
#define DISPLAY_BYTE_PITCH ((DISPLAY_WIDTH + 7) / 8)
#define DISPLAY_BUF_SIZE   (DISPLAY_BYTE_PITCH * DISPLAY_HEIGHT)
LOG_MODULE_REGISTER(main);

#define PIXEL_ON 0
#define PIXEL_OFF 1

static uint8_t raw_buf[DISPLAY_WIDTH * DISPLAY_HEIGHT];  // 1 byte per pixel
static uint8_t buf[DISPLAY_BUF_SIZE];



extern void set_pixel(int x, int y) {
   if (x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT) {
       return;
   }
   raw_buf[y * DISPLAY_WIDTH + x] = PIXEL_ON;
}

static void color_white(int x, int y){
      if (x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT) {
       return;
   }
   raw_buf[y * DISPLAY_WIDTH + x] = PIXEL_OFF;
   
}
static void pack_buffer(void) {
   memset(buf, 0xFF, sizeof(buf));  // Set all to white
   for (int y = 0; y < DISPLAY_HEIGHT; y++) {
       for (int x = 0; x < DISPLAY_WIDTH; x++) {
           if (raw_buf[y * DISPLAY_WIDTH + x] == PIXEL_ON) {
               int byte_index = x + (y / 8) * DISPLAY_WIDTH;
               int bit_pos = 7 - (y % 8);
               buf[byte_index] &= ~(1 << bit_pos);
               // LOG_INF("PACK: (x=%d y=%d) -> Byte %d, bit %d", x, y, byte_index, bit_pos);
           }
       }
   }
}

int main(void)
{

    const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
   const struct device *reset_dev   = device_get_binding("GPIO_1");
   const struct device *spi_dev     = DEVICE_DT_GET(DT_NODELABEL(spi0));
   const struct device *gpio_dev    = DEVICE_DT_GET(DT_NODELABEL(gpio1));


   if (reset_dev) {
       gpio_pin_configure(reset_dev, 3, GPIO_OUTPUT);
       gpio_pin_set(reset_dev, 3, 0);
       k_sleep(K_MSEC(10));
       gpio_pin_set(reset_dev, 3, 1);
       k_sleep(K_MSEC(10));
   }    


   LOG_INF("SPI ready? %d", device_is_ready(spi_dev));
   LOG_INF("GPIO1 ready? %d", device_is_ready(gpio_dev));
   LOG_INF("Chosen display node: %s", DT_NODE_FULL_NAME(DT_CHOSEN(zephyr_display)));


   LOG_INF("Delaying for display power up");
   k_msleep(100);


   if (!device_is_ready(display_dev)) {
       LOG_ERR("Display device not ready");
       return -1;
   }


   LOG_INF("DEVICE IS READY!!!");
   LOG_INF("Display device name: %s", display_dev->name);


   display_blanking_off(display_dev);
   memset(raw_buf, 1, sizeof(raw_buf));  // White pixels


   struct display_buffer_descriptor desc = {
       .width = DISPLAY_WIDTH,
       .height = DISPLAY_HEIGHT,
       .pitch = DISPLAY_PITCH,
       .buf_size = DISPLAY_BUF_SIZE,
   };



    if (!device_is_ready(trig.port) || !device_is_ready(echo.port)) {
        printk("GPIO not ready\n");
        return 0;
    }
      gpio_pin_configure_dt(&trig, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&echo, GPIO_INPUT);
    int count =0;
    float dist_prev=INT32_MIN;

while (1) {
    uint32_t start_ticks = 0, end_ticks = 0;
    uint32_t pulse_duration = 0;
    float distance_cm = 0.0f;

    /* Trigger a 10 µs pulse */
    gpio_pin_set_dt(&trig, 1);
    k_busy_wait(10);
    gpio_pin_set_dt(&trig, 0);

    /* Measure echo pulse width */
    while (gpio_pin_get_dt(&echo) == 0) { /* wait for rising edge */ }
    start_ticks = k_cycle_get_32();

    while (gpio_pin_get_dt(&echo) == 1) { /* wait for falling edge */ }
    end_ticks = k_cycle_get_32();

    pulse_duration = k_cyc_to_us_floor32(end_ticks - start_ticks);
    distance_cm = pulse_duration / 58.0f;  /* HC-SR04 approx: us / 58 = cm */

    /* Print without floats: keep 2 decimals using fixed-point (cm * 100) */
    int cm_x100 = (int)(distance_cm * 100.0f + 0.5f);  // rounded

    /* Update display */
    memset(raw_buf, 0xFF, sizeof(raw_buf));  // clear to white
    char dist_str[50];
    printk("Distance: %d.%02d cm\n", cm_x100 / 100, cm_x100 % 100);

    if (dist_prev != distance_cm) {
    snprintk(dist_str, sizeof(dist_str),
             "Distance is %d.%02d cm", cm_x100 / 100, cm_x100 % 100);

    if (count % 2 == 0) {
        draw_string_8x10(dist_str, 32, 49);
    } else {
        draw_string_8x10(dist_str, 33, 50);
    }

    pack_buffer();
    display_write(display_dev, 0, 0, &desc, buf);
    memset(raw_buf, 0xFF, sizeof(raw_buf));
    count++;

    dist_prev = distance_cm;  // move this inside the condition
}

    k_msleep(200);
}



        return 0;


}
