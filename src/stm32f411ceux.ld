ENTRY(Reset_Handler)

__stack_size = 1K;

MEMORY {
  flash  (rx!w) : ORIGIN = 0x08000000, LENGTH = 512K
  ram    (rw!x) : ORIGIN = 0x20000000, LENGTH = 128K - __stack_size
  stack  (rw!x) : ORIGIN = ORIGIN(ram) + LENGTH(ram), LENGTH = __stack_size
}
