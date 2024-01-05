/*
**      Copyright (c) 2010-2014, United States government as represented by the
**      administrator of the National Aeronautics Space Administration.
**      All rights reserved. This software was created at NASAs Goddard
**      Space Flight Center pursuant to government contracts.
**
**      This is governed by the NASA Open Source Agreement and may be used,
**      distributed and modified only pursuant to the terms of that agreement.
**
*/

/*
** Filename: eefs_filesys.h
**
** Purpose: This file contains the higher level api interface function to the eefs_fileapi.c code.  This layer of the file
**   system supports multiple devices and volumes where the eefs_fileapi.c layer only focuses on a single file system.
**   Most of the functions in this file are essentially wrappers for the lower level function contained in eefs_fileapi.c.
**   All api functions are designed to be as similar to a standard unix filesystem api as possible.
**
** Design Notes:
**
** The Device Table:
**   The device table defines multiple devices or instances of the eeprom file system.  The device table is initialised by
**   calling the function EEFS_InitFS and requires a unique device name and the base address if the file system in eeprom.
**
**   EEFS_InitFS("/EEDEV0", (uint32_t)FileSystemBaseAddress1);
**   EEFS_InitFS("/EEDEV1", (uint32_t)FileSystemBaseAddress2);
**
** The Volume Table:
**   In order to access a device it must be mounted by calling the function EEFS_Mount.  Mounting the device maps a volume name
**   or mount point to the device.
**
**   EEFS_Mount("/EEDEV0", "/EEFS0");
**   EEFS_Mount("/EEDEV1", "/EEFS1");
**
**   Once the file system is mounted then it can be accessed by providing the path of the file as follows:
**   "/MountPoint/Filename"
**
** References:
**
**   See eefs_fileapi.h for additional information.
*/

#ifndef _eefs_filesys_
#define _eefs_filesys_

/*
 * Includes
 */

#include "common_types.h"
#include "eefs_fileapi.h"


#ifdef __cplusplus
extern "C" {
#endif

/*
 * Macro Definitions
 */

#define EEFS_MAX_VOLUMES                2
#define EEFS_MAX_MOUNTPOINT_SIZE        16
#define EEFS_MAX_DEVICES                2
#define EEFS_MAX_DEVICENAME_SIZE        16
#define EEFS_MAX_PATH_SIZE              64

/*
 * Type Definitions
 */

typedef struct
{
    uint32_t                            InUse;
    uint32_t                            BaseAddress;
    uint8_t                             DeviceName[EEFS_MAX_DEVICENAME_SIZE];
    EEFS_InodeTable_t                   InodeTable;
} EEFS_Device_t;

typedef struct
{
    uint32_t                            InUse;
    uint8_t                             DeviceName[EEFS_MAX_DEVICENAME_SIZE];
    uint8_t                             MountPoint[EEFS_MAX_MOUNTPOINT_SIZE];
} EEFS_Volume_t;

typedef struct
{
    uint8_t                             MountPoint[EEFS_MAX_PATH_SIZE];
    uint8_t                             Filename[EEFS_MAX_PATH_SIZE];
} EEFS_SplitPath_t;

/*
 * Exported Functions
 */

/* Initialize global data shared by all file systems */
/* Call this to initialise the EEFS system first, and only do it ONCE */
int8_t							EEFS_Init(void)  __attribute__((flatten));

/* Adds a device to the DeviceTable and calls EEFS_InitFS to initialize the file system.  Note that the DeviceName
 * and the BaseAddress of the file system must be unique. */
int8_t                          EEFS_InitFS(uint8_t *DeviceName, uint32_t BaseAddress);

/* Mounts a device by mapping a MountPoint to a DeviceName.  The device must be initialized prior
   to mounting it by calling EEFS_InitFS. */
int8_t                          EEFS_Mount(uint8_t *DeviceName, uint8_t *MountPoint);

/* Unmounts a volume by removing it from the VolumeTable. */
int8_t                          EEFS_UnMount(uint8_t *MountPoint);

/* Opens the specified file for reading or writing. */
int8_t                          EEFS_Open(uint8_t *Path, uint32_t Flags);

/* Creates a new file and opens it for writing. */
int8_t                          EEFS_Creat(uint8_t *Path, uint32_t Mode);

/* Closes a file. */
int8_t                          EEFS_Close(int8_t FileDescriptor) __attribute((flatten));

/* Read from a file. */
int32_t                         EEFS_Read(int8_t FileDescriptor, void *Buffer, uint32_t Length) __attribute((flatten));

/* Write to a file. */
int32_t                         EEFS_Write(int8_t FileDescriptor, void *Buffer, uint32_t Length) __attribute((flatten));

/* Set the file pointer to a specific offset in the file.  If the ByteOffset is specified that is beyond the end of the
 * file then the file pointer is set to the end of the file.  Currently SEEK_SET, SEEK_CUR, SEEK_END are implemented.
 * Returns the current file pointer on success, or EEFS_ERROR on error.*/
int32_t                         EEFS_LSeek(int8_t FileDescriptor, uint32_t ByteOffset, uint16_t Origin) __attribute((flatten));

/* Removes the specified file from the file system.  Note that this just marks the file as deleted and does not free the memory
 * in use by the file.  Once a file is deleted, the only way the slot can be reused is to manually write a new file into the
 * slot, i.e. there is no way to reuse the memory through a EEFS api function */
int8_t                          EEFS_Remove(uint8_t *Path);

/* Renames the specified file.  Note that you cannot move a file by renaming it to a different volume. */
int8_t                          EEFS_Rename(uint8_t *OldPath, uint8_t *NewPath);

/* Returns file information for the specified file in StatBuffer. */
int8_t                          EEFS_Stat(uint8_t *Path, EEFS_Stat_t *StatBuffer);

/* Sets the Attributes for the specified file. Currently the only attribute that is supported is the EEFS_ATTRIBUTE_READONLY
 * attribute.  To read file attributes use the stat function. */
int8_t                          EEFS_SetFileAttributes(uint8_t *Path, uint32_t Attributes);

/* Opens a Volume for reading the file directory.  This should be followed by calls to EEFS_ReadDir and EEFS_CloseDir. */
EEFS_DirectoryDescriptor_t     *EEFS_OpenDir(uint8_t *MountPoint);

/* Read the next file directory entry.  Returns a pointer to a EEFS_DirectoryEntry_t if successful or NULL if no more file
 * directory entries exist or an error occurs.  Note that all entries are returned, even empty slots. (The InUse flag will be
 * set to FALSE for empty slots) */
EEFS_DirectoryEntry_t          *EEFS_ReadDir(EEFS_DirectoryDescriptor_t *DirectoryDescriptor) __attribute((flatten));

/* Close file system for reading the file directory. */
int8_t                          EEFS_CloseDir(EEFS_DirectoryDescriptor_t *DirectoryDescriptor) __attribute((flatten));

#ifdef __cplusplus
}
#endif

#endif

