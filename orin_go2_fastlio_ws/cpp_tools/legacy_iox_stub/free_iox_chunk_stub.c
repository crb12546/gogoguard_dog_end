#include <stddef.h>

void free_iox_chunk(void *iox_sub, void **iox_chunk)
{
  (void)iox_sub;
  if (iox_chunk != NULL)
  {
    *iox_chunk = NULL;
  }
}

void *iceoryx_header_from_chunk(const void *iox_chunk)
{
  return (void *)iox_chunk;
}