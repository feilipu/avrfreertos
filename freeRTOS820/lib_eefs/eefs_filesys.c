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
** Filename: eefs_filesys.c
**
** Purpose:
**     This file contains the higher level api interface functions to the eefs_fileapi.c code.
**     This layer of the file system supports multiple devices and volumes where the
**     eefs_fileapi.c layer only focuses on a single file system.
**     Most of the functions in this file are essentially wrappers for the lower
**     level function contained in eefs_fileapi.c.
**     All api functions are designed to be as similar to a standard unix filesystem api as possible.
**
*/

/*
 * Includes
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "common_types.h"

#include "eefs_fileapi.h"
#include "eefs_filesys.h"


/*
 * Local Data
 */

static EEFS_Device_t	EEFS_DeviceTable[EEFS_MAX_DEVICES];
static EEFS_Volume_t	EEFS_VolumeTable[EEFS_MAX_VOLUMES];

/*
 * Local Function Prototypes
 */

int8_t                   EEFS_SplitPath(uint8_t *InputPath, EEFS_SplitPath_t *SplitPath) __attribute__((flatten));
EEFS_Device_t           *EEFS_FindDevice(uint8_t *DeviceName) __attribute__((flatten));
EEFS_Device_t           *EEFS_GetDevice(void) __attribute__((flatten));
int8_t                   EEFS_FreeDevice(EEFS_Device_t *Device) __attribute__((flatten));
EEFS_Volume_t           *EEFS_FindVolume(uint8_t *MountPoint) __attribute__((flatten));
EEFS_Volume_t           *EEFS_GetVolume(void) __attribute__((flatten));
int8_t                   EEFS_FreeVolume(EEFS_Volume_t *Volume) __attribute__((flatten));

/*
 * Function Definitions
 */

/* Initialize global data shared by all file systems
 * Call this to initialise the EEFS system first, and only do it ONCE otherwise global data will be reset.
 */
int8_t EEFS_Init(void)
{
	return EEFS_LibInit();
}

/* Adds a device to the DeviceTable and calls EEFS_LibInitFS to initialize the file system.  Note that the DeviceName
 * and the BaseAddress of the file system must be unique. */
int8_t EEFS_InitFS(uint8_t *DeviceName, uint32_t BaseAddress)
{
    EEFS_Device_t               *Device;
    int8_t                       ReturnCode;

    if ((DeviceName != NULL) && (strlen((const char *)DeviceName) < EEFS_MAX_DEVICENAME_SIZE)) {

        if (DeviceName[0] == '/') {

            /* make sure the DeviceName and the BaseAddress are not already configured in the DeviceTable */
            for (uint8_t i=0; i < EEFS_MAX_DEVICES; ++i) {
                if ((EEFS_DeviceTable[i].InUse == TRUE) &&
                   ((strncmp((const char *)EEFS_DeviceTable[i].DeviceName, (const char *)DeviceName, EEFS_MAX_DEVICENAME_SIZE) == 0) &&
                    (EEFS_DeviceTable[i].BaseAddress == BaseAddress)))
                    return(EEFS_DEVICE_ADDRESS_CONFIGURED);
            }

            /* allocate and configure a new device */
            if ((Device = EEFS_GetDevice()) != NULL) {
                Device->BaseAddress = BaseAddress;
                strlcpy((char *)Device->DeviceName, (const char *)DeviceName, EEFS_MAX_DEVICENAME_SIZE);
                if ((ReturnCode = EEFS_LibInitFS(&Device->InodeTable, Device->BaseAddress)) != EEFS_SUCCESS) {
                    EEFS_FreeDevice(Device);
                    ReturnCode = EEFS_ERROR;
                }
            }
            else { /* can't allocate new device */
                ReturnCode = EEFS_DEVICE_CANT_ALLOCATE_NEW;
            }
        }
        else { /* invalid device name */
            ReturnCode = EEFS_DEVICE_INVALID_NAME;
        }
    }
    else { /* device name is too long */
        ReturnCode = EEFS_INVALID_ARGUMENT;
    }

    return ReturnCode;
}

/* Mounts a device by mapping a MountPoint to a DeviceName.  The device must be initialized prior
   to mounting it by calling EEFS_InitFS. */
int8_t EEFS_Mount(uint8_t *DeviceName, uint8_t *MountPoint)
{
    EEFS_Volume_t               *Volume;
    int8_t                       ReturnCode;

    if ((DeviceName != NULL) && (strlen((const char *)DeviceName) < EEFS_MAX_DEVICENAME_SIZE) &&
        (MountPoint != NULL) && (strlen((const char *)MountPoint) < EEFS_MAX_MOUNTPOINT_SIZE)) {

        if ((DeviceName[0] == '/') && (MountPoint[0] == '/')) {

            /* if the DeviceName exists in the DeviceTable */
            if (EEFS_FindDevice(DeviceName) != NULL) {

                /* if the MountPoint does NOT already exist in the VolumeTable */
                if (EEFS_FindVolume(MountPoint) == NULL) {

                    /* allocate and initialize a new volume */
                    if ((Volume = EEFS_GetVolume()) != NULL) {
                        strlcpy((char *)Volume->DeviceName, (const char *)DeviceName, EEFS_MAX_DEVICENAME_SIZE);
                        strlcpy((char *)Volume->MountPoint, (const char *)MountPoint, EEFS_MAX_MOUNTPOINT_SIZE);
                        ReturnCode = EEFS_SUCCESS;
                    }
                    else { /* can't allocate a new volume */
                        ReturnCode = EEFS_VOLUME_CANT_ALLOCATE_NEW;
                    }
                }
                else { /* mount point already exists in the volume table */
                    ReturnCode = EEFS_VOLUME_ALREADY_CONFIGURED;
                }
            }
            else { /* device not found */
                ReturnCode = EEFS_NO_SUCH_DEVICE;
            }
        }
        else { /* invalid volume name */
            ReturnCode = EEFS_VOLUME_INVALID_NAME;
        }
    }
    else { /* volume name is too long */
        ReturnCode = EEFS_VOLUME_NAME_TOO_LONG;
    }

    return ReturnCode;
}

/* Unmounts a volume by removing it from the VolumeTable. */
int8_t EEFS_UnMount(uint8_t *MountPoint)
{
    EEFS_Volume_t               *Volume;
    EEFS_Device_t               *Device;
    int8_t                       ReturnCode;

    if ((MountPoint != NULL) && (strlen((const char *)MountPoint) < EEFS_MAX_MOUNTPOINT_SIZE)) {

        if (MountPoint[0] == '/') {

            if ((Volume = EEFS_FindVolume(MountPoint)) != NULL) {

                if ((Device = EEFS_FindDevice(Volume->DeviceName)) != NULL) {

                    if ((EEFS_LibHasOpenFiles(&Device->InodeTable) == FALSE) &&
                        (EEFS_LibHasOpenDir(&Device->InodeTable) == FALSE)) {

                        ReturnCode = EEFS_FreeVolume(Volume);
                    }
                    else { /* can't unmount a volume while files are open */
                        ReturnCode = EEFS_ERROR;
                    }
                }
                else { /* device not found */
                    ReturnCode = EEFS_NO_SUCH_DEVICE;
                }
            }
            else { /* volume not found */
                ReturnCode = EEFS_NO_SUCH_VOLUME;
            }
        }
        else { /* invalid mount point name */
            ReturnCode = EEFS_ERROR;
        }
    }
    else { /* volume name is NULL */
        ReturnCode = EEFS_ERROR;
    }

    return ReturnCode;
}

/* Opens the specified file for read or write access. */
int8_t EEFS_Open(uint8_t *Path, uint32_t Flags)
{
    EEFS_SplitPath_t             SplitPath;
    EEFS_Volume_t               *Volume;
    EEFS_Device_t               *Device;
    int8_t                       FileDescriptor;
    int8_t                       ReturnCode;

    if (EEFS_SplitPath(Path, &SplitPath) == EEFS_SUCCESS) {

        if ((Volume = EEFS_FindVolume(SplitPath.MountPoint)) != NULL) {

            if ((Device = EEFS_FindDevice(Volume->DeviceName)) != NULL) {

                if ((FileDescriptor = EEFS_LibOpen(&Device->InodeTable, SplitPath.Filename, Flags, 0)) >= 0) {

                    ReturnCode = FileDescriptor;
                }
                else { /* error opening file */
                    ReturnCode = EEFS_ERROR;
                }
            }
            else { /* device not found */
                ReturnCode = EEFS_ERROR;
            }
        }
        else { /* volume not found */
            ReturnCode = EEFS_ERROR;
        }
    }
    else { /* invalid filename */
        ReturnCode = EEFS_ERROR;
    }

    return ReturnCode;
}

/* Closes a file. */
int8_t EEFS_Close(int8_t FileDescriptor)
{
    return EEFS_LibClose(FileDescriptor);
}

/* Read from a file. */
int32_t EEFS_Read(int8_t FileDescriptor, void *Buffer, uint32_t Length)
{
    return EEFS_LibRead(FileDescriptor, Buffer, Length);
}

/* Write to a file. */
int32_t EEFS_Write(int8_t FileDescriptor, void *Buffer, uint32_t Length)
{
    return EEFS_LibWrite(FileDescriptor, Buffer, Length);
}

/* Set the file pointer to a specific offset in the file.  If the ByteOffset is specified that is beyond the end of the
 * file then the file pointer is set to the end of the file.  Currently SEEK_SET, SEEK_CUR, SEEK_END are implemented. */
int32_t EEFS_LSeek(int8_t FileDescriptor, uint32_t ByteOffset, uint16_t Origin)
{
    return EEFS_LibLSeek(FileDescriptor, ByteOffset, Origin);
}

/* Creates a new file and opens it for writing. */
int8_t EEFS_Creat(uint8_t *Path, uint32_t Mode)
{
    EEFS_SplitPath_t             SplitPath;
    EEFS_Volume_t               *Volume;
    EEFS_Device_t               *Device;
    int8_t                       FileDescriptor;
    int8_t                       ReturnCode;

    if (EEFS_SplitPath(Path, &SplitPath) == 0) {

        if ((Volume = EEFS_FindVolume(SplitPath.MountPoint)) != NULL) {

            if ((Device = EEFS_FindDevice(Volume->DeviceName)) != NULL) {

                if ((FileDescriptor = EEFS_LibCreat(&Device->InodeTable, SplitPath.Filename, Mode)) >= 0) {

                    ReturnCode = FileDescriptor;
                }
                else { /* error creating file */
                    ReturnCode = EEFS_ERROR;
                }
            }
            else { /* device not found */
                ReturnCode = EEFS_ERROR;
            }
        }
        else { /* volume not found */
            ReturnCode = EEFS_ERROR;
        }
    }
    else { /* invalid filename */
        ReturnCode = EEFS_ERROR;
    }

    return ReturnCode;
}

/* Removes the specified file from the file system. */
int8_t EEFS_Remove(uint8_t *Path)
{
    EEFS_SplitPath_t             SplitPath;
    EEFS_Volume_t               *Volume;
    EEFS_Device_t               *Device;
    int8_t                      ReturnCode;

    if (EEFS_SplitPath(Path, &SplitPath) == 0) {

        if ((Volume = EEFS_FindVolume(SplitPath.MountPoint)) != NULL) {

            if ((Device = EEFS_FindDevice(Volume->DeviceName)) != NULL) {

                if (EEFS_LibRemove(&Device->InodeTable, SplitPath.Filename) == EEFS_SUCCESS) {

                    ReturnCode = EEFS_SUCCESS;
                }
                else { /* error removing file */
                    ReturnCode = EEFS_ERROR;
                }
            }
            else { /* device not found */
                ReturnCode = EEFS_ERROR;
            }
        }
        else { /* volume not found */
            ReturnCode = EEFS_ERROR;
        }
    }
    else { /* invalid filename */
        ReturnCode = EEFS_ERROR;
    }

    return ReturnCode;
}

/* Renames the specified file.  Note that you cannot move a file by renaming it to a different volume. */
int8_t EEFS_Rename(uint8_t *OldPath, uint8_t *NewPath)
{
    EEFS_SplitPath_t             OldSplitPath;
    EEFS_SplitPath_t             NewSplitPath;
    EEFS_Volume_t               *Volume;
    EEFS_Device_t               *Device;
    int8_t                       ReturnCode;

    if ((EEFS_SplitPath(OldPath, &OldSplitPath) == 0) &&
        (EEFS_SplitPath(NewPath, &NewSplitPath) == 0)) {

        /* the MountPoint for the Old and New Filenames must be the same */
        if (strcmp((char *)OldSplitPath.MountPoint, (const char *)NewSplitPath.MountPoint) == 0) {

            if ((Volume = EEFS_FindVolume(OldSplitPath.MountPoint)) != NULL) {

                if ((Device = EEFS_FindDevice(Volume->DeviceName)) != NULL) {

                    if (EEFS_LibRename(&Device->InodeTable, OldSplitPath.Filename, NewSplitPath.Filename) == EEFS_SUCCESS) {

                        ReturnCode = EEFS_SUCCESS;
                    }
                    else { /* error renaming file */
                        ReturnCode = EEFS_ERROR;
                    }
                }
                else { /* device not found */
                    ReturnCode = EEFS_ERROR;
                }
            }
            else { /* volume not found */
                ReturnCode = EEFS_ERROR;
            }
        }
        else { /* mount points are not the same */
            ReturnCode = EEFS_ERROR;
        }
    }
    else { /* invalid filename */
        ReturnCode = EEFS_ERROR;
    }

    return ReturnCode;
}

/* Returns file information for the specified file in StatBuffer. */
int8_t EEFS_Stat(uint8_t *Path, EEFS_Stat_t *StatBuffer)
{
    EEFS_SplitPath_t             SplitPath;
    EEFS_Volume_t               *Volume;
    EEFS_Device_t               *Device;
    int8_t                       ReturnCode;

    if (EEFS_SplitPath(Path, &SplitPath) == 0) {

        if ((Volume = EEFS_FindVolume(SplitPath.MountPoint)) != NULL) {

            if ((Device = EEFS_FindDevice(Volume->DeviceName)) != NULL) {

                if (EEFS_LibStat(&Device->InodeTable, SplitPath.Filename, StatBuffer) == EEFS_SUCCESS) {

                    ReturnCode = EEFS_SUCCESS;
                }
                else { /* error getting stat */
                    ReturnCode = EEFS_ERROR;
                }
            }
            else { /* device not found */
                ReturnCode = EEFS_ERROR;
            }
        }
        else { /* volume not found */
            ReturnCode = EEFS_ERROR;
        }
    }
    else { /* invalid filename */
        ReturnCode = EEFS_ERROR;
    }

    return ReturnCode;
}

/* Sets the Attributes for the specified file. Currently the only attribute that is supported is the EEFS_ATTRIBUTE_READONLY
 * attribute.  To read file attributes use the stat function. */
int8_t EEFS_SetFileAttributes(uint8_t *Path, uint32_t Attributes)
{
    EEFS_SplitPath_t             SplitPath;
    EEFS_Volume_t               *Volume;
    EEFS_Device_t               *Device;
    int8_t                       ReturnCode;

    if (EEFS_SplitPath(Path, &SplitPath) == 0) {

        if ((Volume = EEFS_FindVolume(SplitPath.MountPoint)) != NULL) {

            if ((Device = EEFS_FindDevice(Volume->DeviceName)) != NULL) {

                if (EEFS_LibSetFileAttributes(&Device->InodeTable, SplitPath.Filename, Attributes) == EEFS_SUCCESS) {

                    ReturnCode = EEFS_SUCCESS;
                }
                else { /* error setting file attributes */
                    ReturnCode = EEFS_ERROR;
                }
            }
            else { /* device not found */
                ReturnCode = EEFS_ERROR;
            }
        }
        else { /* volume not found */
            ReturnCode = EEFS_ERROR;
        }
    }
    else { /* invalid filename */
        ReturnCode = EEFS_ERROR;
    }

    return ReturnCode;
}

/* Opens a Volume for reading the file directory.  This should be followed by calls to EEFS_ReadDir and EEFS_CloseDir. */
EEFS_DirectoryDescriptor_t *EEFS_OpenDir(uint8_t *MountPoint)
{
    EEFS_Device_t               *Device;
    EEFS_Volume_t               *Volume;
    EEFS_DirectoryDescriptor_t  *DirectoryDescriptor;

    if ((MountPoint != NULL) && (strlen((const char *)MountPoint) < EEFS_MAX_MOUNTPOINT_SIZE)) {

        if (MountPoint[0] == '/') {

            if ((Volume = EEFS_FindVolume(MountPoint)) != NULL) {

                if ((Device = EEFS_FindDevice(Volume->DeviceName)) != NULL) {

                    DirectoryDescriptor = EEFS_LibOpenDir(&Device->InodeTable);
                }
                else { /* device not found */
                    DirectoryDescriptor = NULL;
                }
            }
            else { /* volume not found */
                DirectoryDescriptor = NULL;
            }
        }
        else { /* invalid volume name */
            DirectoryDescriptor = NULL;
        }
    }
    else { /* volume name is too long */
        DirectoryDescriptor = NULL;
    }

    return DirectoryDescriptor;
}

/* Read the next file directory entry.  Returns a pointer to a EEFS_DirectoryEntry_t if successful or NULL if no more file
 * directory entries exist or an error occurs.  Note that all entries are returned, even empty slots. (The InUse flag will be
 * set to FALSE for empty slots) */
EEFS_DirectoryEntry_t *EEFS_ReadDir(EEFS_DirectoryDescriptor_t *DirectoryDescriptor)
{
    return EEFS_LibReadDir(DirectoryDescriptor);
}

/* Close file system for reading the file directory. */
int8_t EEFS_CloseDir(EEFS_DirectoryDescriptor_t *DirectoryDescriptor)
{
    return EEFS_LibCloseDir(DirectoryDescriptor);
}

/* Splits a Path into the MountPoint and the Filename and returns the result in SplitPath.  Note that a properly
   formatted Path begins with a '/' followed by the MountPoint or VolumeName, followed by a second '/', followed by
   the Filename.  ex "/MountPoint/Filename" */
int8_t EEFS_SplitPath(uint8_t *InputPath, EEFS_SplitPath_t *SplitPath)
{
	uint32_t 			   i;
	int8_t                 ReturnCode;

    if ((InputPath != NULL) &&
        (SplitPath != NULL)) {

        if (strlen((const char *)InputPath) < EEFS_MAX_PATH_SIZE) {

            memset(SplitPath, '\0', sizeof(EEFS_SplitPath_t));

            if (InputPath[0] == '/') {

                for (i=1; i < strlen((const char *)InputPath); ++i)
                    if (InputPath[i] == '/') break;

                if (i < strlen((const char *)InputPath)) {

                    /* This removes the '/' from the filename */
                    strlcpy((char *)SplitPath->MountPoint, (const char *)InputPath, i);
                    strlcpy((char *)SplitPath->Filename, (const char *)&(InputPath[i+1]), EEFS_MAX_PATH_SIZE);
                    ReturnCode = EEFS_SUCCESS;
                }
                else { /* missing second '/' */
                    ReturnCode = EEFS_ERROR;
                }
            }
            else { /* missing first '/' */
                ReturnCode = EEFS_ERROR;
            }
        }
        else { /* filename too long */
            ReturnCode = EEFS_ERROR;
        }
    }
    else { /* Invalid pointer */
        ReturnCode = EEFS_ERROR;
    }

    return ReturnCode;
}

/* Performs a sequential search of the DeviceTable looking for a matching DeviceName. */
EEFS_Device_t *EEFS_FindDevice(uint8_t *DeviceName)
{
    for (uint32_t i=0; i < EEFS_MAX_DEVICES; i++) {
        if ((EEFS_DeviceTable[i].InUse == TRUE) &&
            (strncmp((const char *)DeviceName, (const char *)EEFS_DeviceTable[i].DeviceName, EEFS_MAX_DEVICENAME_SIZE) == 0))
            return(&EEFS_DeviceTable[i]);
    }
    return NULL;
}

/* Allocates a free entry in the DeviceTable. */
EEFS_Device_t *EEFS_GetDevice(void)
{
    for (uint32_t i=0; i < EEFS_MAX_DEVICES; ++i) {
        if (EEFS_DeviceTable[i].InUse == FALSE) {
            EEFS_DeviceTable[i].InUse = TRUE;
            return(&EEFS_DeviceTable[i]);
        }
    }
    return NULL;
}

/* Returns a entry to the DeviceTable. */
int8_t EEFS_FreeDevice(EEFS_Device_t *Device)
{
    if (Device != NULL) {
        memset(Device, 0, sizeof(EEFS_Device_t));
        return EEFS_SUCCESS;
    }
    return EEFS_ERROR;
}

/* Performs a sequential search of the VolumeTable looking for a matching MountPoint. */
EEFS_Volume_t *EEFS_FindVolume(uint8_t *MountPoint)
{
    for (uint32_t i=0; i < EEFS_MAX_VOLUMES; ++i) {
        if ((EEFS_VolumeTable[i].InUse == TRUE) &&
            (strncmp((const char *)MountPoint, (const char *)EEFS_VolumeTable[i].MountPoint, EEFS_MAX_MOUNTPOINT_SIZE) == 0))
            return(&EEFS_VolumeTable[i]);
    }
    return NULL;
}

/* Allocates a free entry in the VolumeTable. */
EEFS_Volume_t *EEFS_GetVolume(void)
{
    for (uint32_t i=0; i < EEFS_MAX_VOLUMES; ++i) {
        if (EEFS_VolumeTable[i].InUse == FALSE) {
            EEFS_VolumeTable[i].InUse = TRUE;
            return(&EEFS_VolumeTable[i]);
        }
    }
    return NULL;
}

/* Returns a entry to the VolumeTable. */
int8_t EEFS_FreeVolume(EEFS_Volume_t *Volume)
{
    if (Volume != NULL) {
        memset(Volume, 0, sizeof(EEFS_Volume_t));
        return(EEFS_SUCCESS);
    }
    return EEFS_ERROR;
}
