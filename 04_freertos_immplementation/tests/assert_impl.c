#include <stdio.h>
#include <stdlib.h>

void vAssertCalled(const char *pcFile, int iLine)
{
    fprintf(stderr, "ASSERT FAILED: %s:%d\n", pcFile, iLine);
    fflush(stderr);
    abort();
}
