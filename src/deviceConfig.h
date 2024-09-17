
/* Configuration for each devive */

/* Set the device ID for deployment, later to be replaced by compiler argument */
#define DEVICE_NR 1

#if DEVICE_NR==1

  #define NWKSKEY { 0x39, 0x6F, 0x74, 0x5C, 0x6A, 0x94, 0xD7, 0x0F, 0xD4, 0x59, 0xCF, 0x3B, 0x8B, 0x05, 0x66, 0xF6 }
  #define APPSKEY { 0xCF, 0x09, 0x98, 0x84, 0x06, 0x85, 0xFF, 0x05, 0x16, 0x30, 0x6E, 0x76, 0xD1, 0xA2, 0x0D, 0xF5 }
  #define DEVADDR 0x260B15B4

#endif
