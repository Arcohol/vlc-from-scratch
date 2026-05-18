#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "vlc_tx.h"

int main(void) {
  printk("oh-my-dmd\n");

  int ret = vlc_tx_init();

  if (ret != 0) {
    printk("VLC TX init failed: %d\n", ret);
    return ret;
  }

  printk("VLC TX ready\n");

  uint32_t sequence = 0;
  char message[64];

  while (true) {
    // Send START to indicate start of transmission
    int len = snprintk(message, sizeof(message), "START");
    if (len < 0 || (size_t)len >= sizeof(message)) {
      printk("Message formatting failed: %d\n", len);
      return -EINVAL;
    }
    int ret = vlc_tx_send((const uint8_t *)message, (uint32_t)len, K_FOREVER);
    if (ret != 0) {
      printk("VLC TX send failed: %d\n", ret);
      return ret;
    }

    sequence = 0;
    for (int i = 0; i < 1000; ++i) {
      int len = snprintk(message, sizeof(message), "Hello, world! seq=%lu",
                         (unsigned long)sequence);

      if (len < 0 || (size_t)len >= sizeof(message)) {
        printk("Message formatting failed: %d\n", len);
        return -EINVAL;
      }

      ret = vlc_tx_send((const uint8_t *)message, (uint32_t)len, K_FOREVER);

      if (ret != 0) {
        printk("VLC TX send failed: %d\n", ret);
        return ret;
      }

      ret = vlc_tx_get_error();

      if (ret != 0) {
        printk("VLC TX failed: %d\n", ret);
        return ret;
      }

      printk("Sent: %s\n", message);

      ++sequence;
      k_msleep(5);
    }

    // Send STOP to indicate end of transmission
    len = snprintk(message, sizeof(message), "STOP");
    if (len < 0 || (size_t)len >= sizeof(message)) {
      printk("Message formatting failed: %d\n", len);
      return -EINVAL;
    }
    ret = vlc_tx_send((const uint8_t *)message, (uint32_t)len, K_FOREVER);
    if (ret != 0) {
      printk("VLC TX send failed: %d\n", ret);
      return ret;
    }
    k_msleep(3000);
  }
}
