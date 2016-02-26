
#include <stdint.h>
#include <stdlib.h>

/*-----------------------------------------------------------*/
int16_t main(void) __attribute__((OS_main));

int16_t main(void)
{
	setup();
}

int a = -10;
int b = 10;
long x;

void setup() {
  x = (long) a * b;
}
