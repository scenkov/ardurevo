
#include "Semaphores.h"

using namespace VRBRAIN;

VRBRAINSemaphore::VRBRAINSemaphore()
  : m_semaphore(NULL)
{
}

void VRBRAINSemaphore::init()
{
  m_semaphore = xSemaphoreCreateMutex();
}

bool VRBRAINSemaphore::take(uint32_t timeout_ms)
{
  portTickType delay;

  if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER)
    delay = portMAX_DELAY;
  else
    delay = timeout_ms / portTICK_RATE_MS;

  return xSemaphoreTake(m_semaphore, delay);
}

bool VRBRAINSemaphore::take_nonblocking()
{
  return xSemaphoreTake(m_semaphore, 0);
}

bool VRBRAINSemaphore::give()
{
  return xSemaphoreGive(m_semaphore);
}
