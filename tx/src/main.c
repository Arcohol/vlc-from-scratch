#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "vlc_tx.h"

int main(void) {
  printk("oh-my-dmd\n");

  int ret = vlc_tx_init();

  if (ret != 0) {
    printk("VLC TX init failed: %d\n", ret);
    return ret;
  }

  printk("VLC TX ready\n");

  static const char *const messages[] = {
      "hello",
      "dmd",
      "vlc",
  };

  while (true) {
    for (size_t i = 0; i < ARRAY_SIZE(messages); ++i) {
      const char *message = messages[i];

      ret = vlc_tx_send((const uint8_t *)message, strlen(message), K_FOREVER);

      if (ret != 0) {
        printk("VLC TX send failed: %d\n", ret);
        return ret;
      }

      ret = vlc_tx_get_error();

      if (ret != 0) {
        printk("VLC TX failed: %d\n", ret);
        return ret;
      }
    }
  }
}
