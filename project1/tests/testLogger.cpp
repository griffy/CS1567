#include "../logger.h"

int main() {
    LOG.setImportanceLevel(LOG_HIGH);

    float x = 10.1;
    float y = 12.1;
    float theta = 0.45;

    LOG.write(LOG_HIGH, "ns raw", "%f,%f,%f", x, y, theta);
    LOG.write(LOG_HIGH, "ns raw", "%f,%f,%f", x, y, theta);
    LOG.write(LOG_MED, "ns raw", "%f,%f,%f", x, y, theta);
    LOG.write(LOG_HIGH, "we raw", "%f,%f,%f", x, y, theta);
    LOG.write(LOG_HIGH, "we raw", "%d", 10);
    LOG.write(LOG_HIGH, "we raw", "%s", "Testing...");

    return 0;
}

