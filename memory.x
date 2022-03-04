MEMORY
{
  /* TODO Adjust these memory regions to match your device memory layout */
  /* These values correspond to the STM32F413CHU*/
  FLASH : ORIGIN = 0x08000000, LENGTH = 1536K
  RAM : ORIGIN = 0x20000000, LENGTH = 320K
}

_stext = ORIGIN(FLASH) + 0x8000;
