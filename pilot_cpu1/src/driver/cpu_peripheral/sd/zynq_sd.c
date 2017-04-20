
#include <compiler.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sdio.h>
#include <string.h>
#include <errno.h>

#include <fs/fs.h>
#include <fs/ioctl.h>

#include "xparameters.h"
#include "xil_types.h"

#include "xsdps.h"		

/* 
 * The maximum number of references on the driver (because a uint8_t is used.
 * Use a larger type if more references are needed.
 */

#define MAX_CREFS               0xff

#define SDIO_STATUS_NOINIT      0x01U	/* Drive not initialized */
#define SDIO_STATUS_PRESENT     0x02U	/* No medium in the drive */
#define SDIO_STATUS_WRPROTECTED 0x04U	/* Write protected */
#define mmcsd_givesem(p) sem_post(&priv->sem)

typedef struct{
	int32_t slot_id;	//uartns port id
	XSdPs *p_instance;
    sem_t  sem;                      /* Assures mutually exclusive access to the slot */
    uint8_t  crefs;                  /* Open references on the driver */

  /* Memory card geometry (extracted from the CSD) */
    uint8_t  blockshift;             /* Log2 of blocksize */
    uint16_t blocksize;              /* Read block length (== block size) */
    uint32_t nblocks;                /* Number of blocks */

    uint64_t capacity;               /* Total capacity of volume */

}sd_private_t;

/* Block driver methods *****************************************************/

static int     mmcsd_open(FAR struct inode *inode);
static int     mmcsd_close(FAR struct inode *inode);
static ssize_t mmcsd_read(FAR struct inode *inode, FAR unsigned char *buffer, size_t startsector, unsigned int nsectors);
static ssize_t mmcsd_write(FAR struct inode *inode, const unsigned char *buffer, size_t startsector, unsigned int nsectors);
static int     mmcsd_geometry(FAR struct inode *inode, struct geometry *geometry);
static int     mmcsd_ioctl(FAR struct inode *inode, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static XSdPs sd_instance[XPAR_XSDPS_NUM_INSTANCES] = {0};

static const struct block_operations g_bops =
{
  mmcsd_open,     /* open     */
  mmcsd_close,    /* close    */
  mmcsd_read,     /* read     */
  mmcsd_write,    /* write    */
  mmcsd_geometry, /* geometry */
  mmcsd_ioctl     /* ioctl    */
};

static void mmcsd_takesem(sd_private_t *priv)
{
  /* Take the semaphore, giving exclusive access to the driver (perhaps
   * waiting)
   */

    while (sem_wait(&priv->sem) != 0)
    {
        /* The only case that an error should occur here is if the wait was
        * awakened by a signal.
        */

        DEBUGASSERT(errno == EINTR);
    }
}

static uint8_t mmcsd_status(sd_private_t *priv)
{
    uint32_t reg = 0;
    uint8_t status = SDIO_STATUS_NOINIT;

	reg = XSdPs_GetPresentStatusReg((uint32_t)priv->p_instance->Config.BaseAddress);
    if(reg & XSDPS_PSR_CARD_INSRT_MASK) 
    {
        status |= SDIO_STATUS_PRESENT;
        status &= ~(SDIO_STATUS_NOINIT);
    }

	if ((reg & XSDPS_PSR_WPS_PL_MASK) == 0U)
        status |= SDIO_STATUS_WRPROTECTED;
    else
        status &= ~(SDIO_STATUS_WRPROTECTED);

    return status;
} 

static int mmcsd_open(FAR struct inode *inode)
{
	sd_private_t *priv = NULL;

    DEBUGASSERT(inode && inode->i_private);
    priv = (sd_private_t *)inode->i_private;
    /* Just increment the reference count on the driver */

    DEBUGASSERT(priv->crefs < MAX_CREFS);
    mmcsd_takesem(priv);
    priv->crefs++;
    mmcsd_givesem(priv);

    return OK;

}

/****************************************************************************
 * Name: mmcsd_close
 *
 * Description: close the block device
 *
 ****************************************************************************/
static int mmcsd_close(FAR struct inode *inode)
{
	sd_private_t *priv = NULL;

    DEBUGASSERT(inode && inode->i_private);
    priv = (FAR sd_private_t *)inode->i_private;

    DEBUGASSERT(priv->crefs > 0);
    mmcsd_takesem(priv);
    priv->crefs--;
    mmcsd_givesem(priv);

    return OK;

}

/****************************************************************************
 * Name: mmcsd_read
 *
 * Description:
 *   Read the specified numer of sectors from the read-ahead buffer or from
 *   the physical device.
 *
 ****************************************************************************/

static ssize_t mmcsd_read(struct inode *inode, unsigned char *buffer, size_t startsector, unsigned int nsectors)
{
    uint8_t status;
	sd_private_t *priv = NULL;
	size_t loc_sector = startsector;

    DEBUGASSERT(inode && inode->i_private);
    priv = (FAR sd_private_t *)inode->i_private;

    mmcsd_takesem(priv);
    status = mmcsd_status(priv);
	if ((status & SDIO_STATUS_NOINIT) != 0U) {
        mmcsd_givesem(priv);
		return 0;
	}
	if (nsectors == 0U) {
        mmcsd_givesem(priv);
		return 0;
	}

	/* Convert LBA to byte address if needed */
	if ((priv->p_instance->HCS) == 0U) {
		loc_sector *= (size_t)XSDPS_BLK_SIZE_512_MASK;
	}

    //pilot_info("loc_sector=%x instance=%x hcs=%x\n", (int)loc_sector, (int)priv->p_instance, (int)priv->p_instance->HCS);
	status = XSdPs_ReadPolled(priv->p_instance, (uint32_t)loc_sector, nsectors, buffer);
    if(status != XST_SUCCESS)
    {
        mmcsd_givesem(priv);
        return 0;
    }

    mmcsd_givesem(priv);

    return nsectors;
}

static ssize_t mmcsd_write(FAR struct inode *inode, FAR const unsigned char *buffer, size_t startsector, unsigned int nsectors)
{
    uint8_t status;
	sd_private_t *priv = NULL;
	size_t loc_sector = startsector;

    DEBUGASSERT(inode && inode->i_private);
    priv = (FAR sd_private_t *)inode->i_private;

    mmcsd_takesem(priv);
    status = mmcsd_status(priv);
	if ((status & SDIO_STATUS_NOINIT) != 0U) {
        mmcsd_givesem(priv);
		return 0;
	}
	if (nsectors == 0U) {
        mmcsd_givesem(priv);
		return 0;
	}

	/* Convert LBA to byte address if needed */
  	if ((priv->p_instance->HCS) == 0U) 
  	{
		loc_sector *= (size_t)XSDPS_BLK_SIZE_512_MASK;
	}

    //pilot_info("loc_sector=%x instance=%x hcs=%x\n", (int)loc_sector, (int)priv->p_instance, (int)priv->p_instance->HCS);
	status = XSdPs_WritePolled(priv->p_instance, (uint32_t)loc_sector, nsectors, buffer);
    if(status != XST_SUCCESS)
    {
        mmcsd_givesem(priv);
        return 0;
    }

    mmcsd_givesem(priv);

    return nsectors;
}

/****************************************************************************
 * Name: mmcsd_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/
static int mmcsd_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{

    return 0;
}

/****************************************************************************
 * Name: mmcsd_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/
static int mmcsd_geometry(FAR struct inode *inode, struct geometry *geometry)
{
    uint8_t status;
	sd_private_t *priv = NULL;
    int ret = 0;

    DEBUGASSERT(inode && inode->i_private);
    priv = (FAR sd_private_t *)inode->i_private;
    
    if(geometry)
    {
        mmcsd_takesem(priv);
        status = mmcsd_status(priv);
        if(status & SDIO_STATUS_NOINIT != 0)
        {
            pilot_err("no sd card\n");
            ret = -ENODEV;
        }
        else
        {
            geometry->geo_available     = true;
            geometry->geo_mediachanged  = false;
            geometry->geo_writeenabled  = ((status & SDIO_STATUS_WRPROTECTED) != 0)?false:true;
            geometry->geo_nsectors      = priv->p_instance->SectorCount;
            geometry->geo_sectorsize    = XSDPS_BLK_SIZE_512_MASK;
        
        }
        mmcsd_givesem(priv);
    }

    return ret;
}

static uint32_t sdio_ps_clk_init(int32_t slotno)
{    
//SDIO时钟使能
#define SDIO_CLK_CTRL       (*(volatile unsigned long *)(0xf8000150))
#define SDIO0_CLK_ENABLE    (1 << 0)
#define SDIO1_CLK_ENABLE    (1 << 1)

//外设时钟使能
#define APER_CLK_CTRL       (*(volatile unsigned long *)(0xf800012c)) //AMBA Peripheral Clock Control
#define SDI0_CPU_1XCLKACT  (1 << 10)
#define SDI1_CPU_1XCLKACT  (1 << 11)

    pilot_info("1:SDIO%d SDIO_CLK_CTRL=%x APER_CLK_CTRL=%x\n", slotno, SDIO_CLK_CTRL, APER_CLK_CTRL);
    if(slotno == 0) 
    {
        //SDIO clock
        SDIO_CLK_CTRL |= SDIO0_CLK_ENABLE;

        //peripheral clock
        APER_CLK_CTRL |= SDI0_CPU_1XCLKACT;
    }
    else if(slotno == 1)
    {
        //SDIO clock
        SDIO_CLK_CTRL |= SDIO1_CLK_ENABLE;

        //peripheral clock
        APER_CLK_CTRL |= SDI1_CPU_1XCLKACT;
    }

    pilot_info("SDIO%d SDIO_CLK_CTRL=%x APER_CLK_CTRL=%x\n", slotno, SDIO_CLK_CTRL, APER_CLK_CTRL);

    return 0;
}
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmcsd_initialize
 *
 * Description:
 *   Initialize one slot for operation using the MMC/SD interface
 *
 * Input Parameters:
 *   slotno - sd card slot id
 *
 ****************************************************************************/
int mmcsd_initialize(int slotno)
{
	XSdPs_Config *sd_config;
    uint32_t status = 0;
    char devname[16];
	XSdPs *p_instance = &sd_instance[slotno];
	sd_private_t *private = NULL;
    int32_t ret = 0;

    /* enable clock */
    sdio_ps_clk_init(slotno);

	private = (sd_private_t*)pvPortMalloc(sizeof(sd_private_t));
	if(private == NULL)
	{
		pilot_err("malloc failed!!\n");
		return XST_FAILURE;
	}
	else
	{
		memset(private, 0, sizeof(sd_private_t));
	}

    sem_init(&private->sem, 0, 1);
    private->slot_id = slotno;
	private->p_instance = p_instance;

	sd_config = XSdPs_LookupConfig((uint16_t)slotno);
	if (NULL == sd_config) {
		return -1;
	}

	status = XSdPs_CfgInitialize(p_instance, sd_config, sd_config->BaseAddress);
	if (status != XST_SUCCESS) {
		return -1;
	}

	status = XSdPs_CardInitialize(p_instance);
	if (status != XST_SUCCESS) {
		return -1;
	}

    /* Create a MMCSD device name */
    snprintf(devname, 16, "/dev/mmcsd%d", slotno);

    /* Inode private data is a reference to the MMCSD state structure */

    ret = register_blockdriver(devname, &g_bops, 0, private);
    if (ret < 0)
    {
        pilot_err("ERROR: register_blockdriver failed: %d\n", ret);
        return -1;
    }

    return ret;
}

