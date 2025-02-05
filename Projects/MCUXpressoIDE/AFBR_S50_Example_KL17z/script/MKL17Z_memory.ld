/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright (c) 2008-2013 Code Red Technologies Ltd,
 * Copyright 2015, 2018-2019 NXP
 * (c) NXP Semiconductors 2013-2024
 * Generated linker script file for MKL17Z256xxx4
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v11.5.0 [Build 7232] [2022-01-11] on Feb 14, 2024, 2:15:57 PM
 */

MEMORY
{
  /* Define each memory region */
  PROGRAM_FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x3ec00 /* 251K bytes (alias Flash) */  
  USER_DATA_FLASH (rx) : ORIGIN = 0x3ec00, LENGTH = 0x1400 /* 5K bytes (alias Flash2) */  
  SRAM (rwx) : ORIGIN = 0x1fffe000, LENGTH = 0x8000 /* 32K bytes (alias RAM) */  
}

  /* Define a symbol for the top of each memory region */
  __base_PROGRAM_FLASH = 0x0  ; /* PROGRAM_FLASH */  
  __base_Flash = 0x0 ; /* Flash */  
  __top_PROGRAM_FLASH = 0x0 + 0x3ec00 ; /* 251K bytes */  
  __top_Flash = 0x0 + 0x3ec00 ; /* 251K bytes */  
  __base_USER_DATA_FLASH = 0x3ec00  ; /* USER_DATA_FLASH */  
  __base_Flash2 = 0x3ec00 ; /* Flash2 */  
  __top_USER_DATA_FLASH = 0x3ec00 + 0x1400 ; /* 5K bytes */  
  __top_Flash2 = 0x3ec00 + 0x1400 ; /* 5K bytes */  
  __base_SRAM = 0x1fffe000  ; /* SRAM */  
  __base_RAM = 0x1fffe000 ; /* RAM */  
  __top_SRAM = 0x1fffe000 + 0x8000 ; /* 32K bytes */  
  __top_RAM = 0x1fffe000 + 0x8000 ; /* 32K bytes */  
