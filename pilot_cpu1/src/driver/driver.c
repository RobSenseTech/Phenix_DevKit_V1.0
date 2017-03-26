#include "driver.h"
#include "timers.h"

typedef struct xDRIVER_DESCREPTOR
{
	struct list_head xDriverListNode;//驱动队列节点
	char cName[MAX_DRIVER_NAME_LEN];
	int iDriverId;
	int iOpenFlag;//是否驱动已经被打开
	xSemaphoreHandle xOpenMutex;
	xSemaphoreHandle xCloseMutex;
	xSemaphoreHandle xReadMutex;
	xSemaphoreHandle xWriteMutex;
	xSemaphoreHandle xIoctlMutex;
	xSemaphoreHandle xPollMutex;
	xSemaphoreHandle xSeekMutex;
	const DriverOps_t *pxDriverOps;
    Handle_t xHandle;
} DriverDescreptor_t ;

static xSemaphoreHandle xDriverMutex;//互斥锁
//static DriverDescreptor_t *pxDrvDesc[MAX_DRIVER_NUM]={NULL};
static struct list_head xDriverList;//链表头

int DriverManagerInit()
{
	int iRet = -1;

	//初始化互斥锁
	xDriverMutex = xSemaphoreCreateMutex();
	if(xDriverMutex != NULL)
	{
		iRet = 0;
	}
    else
    {
        iRet = -1;
        Print_Err("Create driver mutex failed!!\n");
    }

	//初始化驱动队列
	INIT_LIST_HEAD(&xDriverList);	

	return iRet;
}

//判断节点是否存在，存在返回0，否则返回-1
int DriverDevExists(const char *pcDrvName)
{
	DriverDescreptor_t *pTmpDriverDesc;

	xSemaphoreTake(xDriverMutex, portMAX_DELAY);
    if(pcDrvName == NULL)
	{
		Print_Err("Illegal driver name!!\n");
	    xSemaphoreGive(xDriverMutex);
        return -1;
	}

    //遍历链表查找重名驱动,若有重名，返回失败
	list_for_each_entry(pTmpDriverDesc,&xDriverList,xDriverListNode)
	{
		if(strcmp(pTmpDriverDesc->cName, pcDrvName) == 0)
		{
	        xSemaphoreGive(xDriverMutex);
            return 0; 
		}
	}

	xSemaphoreGive(xDriverMutex);

    return -1;
}

//驱动注册函数，若有同名驱动会自动加上数字后缀,默认后缀为0
int DriverRegister(const char *pcDrvName, const DriverOps_t *pOps, void *pvPrivate)
{
	static int iCurrDrvNum = 0;
	int iRet = 0;
	DriverDescreptor_t *pTmpDriverDesc;

	xSemaphoreTake(xDriverMutex, portMAX_DELAY);

	if(pcDrvName == NULL)
	{
		Print_Err("Illegal driver name!!\n");
		goto OUT;
	}

	if(strlen(pcDrvName) >= MAX_DRIVER_NAME_LEN)
	{
		Print_Err("Driver name is too long,it should be shorter than 39\n");
		iRet = -EINVAL;
		goto OUT;
	}

	//遍历链表查找重名驱动,若有重名，返回失败
	list_for_each_entry(pTmpDriverDesc,&xDriverList,xDriverListNode)
	{
		if(strcmp(pTmpDriverDesc->cName, pcDrvName) == 0)
		{
            iRet = -EEXIST;
            goto OUT;
		}
	}

	if(pOps == NULL)
	{
		Print_Err("Invalid Driver Operation\n");
		iRet = -EINVAL;
		goto OUT;
	}

	//生成驱动描述结构体
	pTmpDriverDesc = (DriverDescreptor_t *)pvPortMalloc(sizeof(DriverDescreptor_t));
	memset(pTmpDriverDesc, 0, sizeof(DriverDescreptor_t));
	strcpy(pTmpDriverDesc->cName, pcDrvName);
	pTmpDriverDesc->iDriverId = iCurrDrvNum;
	pTmpDriverDesc->xOpenMutex = xSemaphoreCreateMutex();
	pTmpDriverDesc->xCloseMutex = xSemaphoreCreateMutex();
	pTmpDriverDesc->xReadMutex = xSemaphoreCreateMutex();
	pTmpDriverDesc->xWriteMutex = xSemaphoreCreateMutex();
	pTmpDriverDesc->xIoctlMutex = xSemaphoreCreateMutex();
	pTmpDriverDesc->xPollMutex = xSemaphoreCreateMutex();
	pTmpDriverDesc->xSeekMutex= xSemaphoreCreateMutex();
	pTmpDriverDesc->pxDriverOps = pOps;
	if(pvPrivate != NULL)
		pTmpDriverDesc->xHandle.xInode.pvPrivate = pvPrivate;

	list_add_tail(&pTmpDriverDesc->xDriverListNode, &xDriverList);//节点加入链表尾
//	Print_Info("%s regist ok, currDevNum=%d pTmpDriverDesc=%x pvPrivate=%x\n", pTmpDriverDesc->cName, iCurrDrvNum, (int)pTmpDriverDesc, pTmpDriverDesc->xHandle.xInode.pvPrivate);

	iCurrDrvNum++;

OUT:
	xSemaphoreGive(xDriverMutex);
    return iRet;
}


int DriverUnregister(const char *pcDrvName)
{
    int iFind = 0;
	DriverDescreptor_t *pTmpDriverDesc = NULL;

    Print_Info("delete dev name:%s\n", pcDrvName);

	xSemaphoreTake(xDriverMutex, portMAX_DELAY);
	list_for_each_entry(pTmpDriverDesc,&xDriverList,xDriverListNode)
	{
		if(strcmp(pTmpDriverDesc->cName, pcDrvName) == 0)
		{
            Print_Info("find dev:%s, delete\n", pcDrvName);
            iFind = 1;
			break;
		}
	}

    if(iFind == 0)
    {
        Print_Err("No such driver, check dev name!!\n");
	    xSemaphoreGive(xDriverMutex);
        return -ENXIO;
    }

	list_del(&pTmpDriverDesc->xDriverListNode);
	vPortFree(pTmpDriverDesc);
	xSemaphoreGive(xDriverMutex);
	return 0;
}

//根据名字查找驱动，并返回驱动描述符
int open(const char *pcName, int iFlag)
{
	int iDriverId = -1;
	DriverDescreptor_t *pTmpDriverDesc = NULL;

    if(strlen(pcName) >= MAX_DRIVER_NAME_LEN)
    {
        Print_Info("name is too long!!\n");
        return -1;
    }

	xSemaphoreTake(xDriverMutex, portMAX_DELAY);
	list_for_each_entry(pTmpDriverDesc,&xDriverList,xDriverListNode)
	{
		//寻找名称一样的驱动名
		if(strcmp(pTmpDriverDesc->cName, pcName) == 0)
		{
          //  Print_Info("Find :[%s, %x] open it, pvPrivate=%x\n", pcName, (int)pTmpDriverDesc, (int)pTmpDriverDesc->xHandle.xInode.pvPrivate);
			iDriverId = pTmpDriverDesc->iDriverId;
			break;	
		}
	}
	xSemaphoreGive(xDriverMutex);

	if(iDriverId < 0)
	{
//		Print_Err("No such Driver:%s\n!!", pcName);
		return -1;
	}

	xSemaphoreTake(pTmpDriverDesc->xOpenMutex, portMAX_DELAY);
	pTmpDriverDesc->xHandle.iFlag = iFlag;
	pTmpDriverDesc->xHandle.iDriverId = iDriverId;
	if(pTmpDriverDesc->pxDriverOps->open != NULL)
	{
		pTmpDriverDesc->pxDriverOps->open(&pTmpDriverDesc->xHandle);//调用驱动私有open函数
	}

	pTmpDriverDesc->iOpenFlag++;
	xSemaphoreGive(pTmpDriverDesc->xOpenMutex);
	
	return (int)pTmpDriverDesc;
}

int close(int iFd)
{
	DriverDescreptor_t *pTmpDriverDesc = (DriverDescreptor_t *)iFd;

    if(iFd == -1 || pTmpDriverDesc->pxDriverOps == NULL)
	{
		Print_Err("Invalid Operation\n");
		return -1;
	}

	xSemaphoreTake(pTmpDriverDesc->xCloseMutex, portMAX_DELAY);
	if(pTmpDriverDesc->pxDriverOps->close != NULL)
	{
		pTmpDriverDesc->pxDriverOps->close(&pTmpDriverDesc->xHandle);//调用驱动私有close函数
	}

    if(pTmpDriverDesc->iOpenFlag != 0)
    	pTmpDriverDesc->iOpenFlag --;
	
	xSemaphoreGive(pTmpDriverDesc->xCloseMutex);
	return 0;
}
size_t read(int iFd, char *pcBuffer, size_t xBufLen)
{
	int iReadLen = 0;
	DriverDescreptor_t *pTmpDriverDesc = (DriverDescreptor_t *)iFd;

	if(iFd == -1 || pTmpDriverDesc->pxDriverOps == NULL)
	{
		Print_Err("Invalid Operation\n");
		goto END;
	}
	
	xSemaphoreTake(pTmpDriverDesc->xReadMutex, portMAX_DELAY);

	if(pTmpDriverDesc->pxDriverOps->read != NULL)
	{
		iReadLen = pTmpDriverDesc->pxDriverOps->read(&pTmpDriverDesc->xHandle, pcBuffer, xBufLen);
	}
	else
	{
		Print_Err("Invalid Operation\n");
		goto END;	
	}

END:
	xSemaphoreGive(pTmpDriverDesc->xReadMutex);
	return iReadLen;
}


size_t write(int iFd, const char *pcBuffer, size_t xBufLen)
{
	int iWriteLen = 0;
	DriverDescreptor_t *pTmpDriverDesc = (DriverDescreptor_t *)iFd;
	
	if(iFd == -1 && pTmpDriverDesc->pxDriverOps == NULL)
	{
		Print_Err("Invalid Operation\n");
		goto END;
	}

	xSemaphoreTake(pTmpDriverDesc->xWriteMutex, portMAX_DELAY);

	if(pTmpDriverDesc->pxDriverOps->write != NULL)
	{
			iWriteLen = pTmpDriverDesc->pxDriverOps->write(&pTmpDriverDesc->xHandle, pcBuffer, xBufLen);
	}
	else
	{
		Print_Err("Invalid Operation\n");
		goto END;
	}

END:
	xSemaphoreGive(pTmpDriverDesc->xWriteMutex);
	return iWriteLen;
}

	
int ioctl(int iFd, int iCmd, void *pvArg)
{
	int iRet = -1;
	DriverDescreptor_t *pTmpDriverDesc = (DriverDescreptor_t *)iFd;

	if(iFd == -1 && pTmpDriverDesc->pxDriverOps == NULL)
	{
		Print_Err("Invalid Operation\n");
		goto END;
	}

	xSemaphoreTake(pTmpDriverDesc->xIoctlMutex, portMAX_DELAY);

	if(pTmpDriverDesc->pxDriverOps->ioctl != NULL)
	{
		iRet = pTmpDriverDesc->pxDriverOps->ioctl(&pTmpDriverDesc->xHandle, iCmd, pvArg);
	}
	else
	{
		Print_Err("Invalid Operation\n");
		goto END;
	}

END:
	xSemaphoreGive(pTmpDriverDesc->xIoctlMutex);
	return iRet;
}

/****************************************************************************
 * Name: poll_timeout
 *
 * Description:
 *   The wdog expired before any other events were received.
 *
 ****************************************************************************/
static void poll_timeout(xTimerHandle xTimer, void *pvArg)
{
    SemaphoreHandle_t sem = (SemaphoreHandle_t)pvArg;

 //   Print_Info("Times up, unlock\n");
    xSemaphoreGive(sem);
}

/****************************************************************************
 * Name: poll_fdsetup
 *
 * Description:
 *   Configure (or unconfigure) one file/socket descriptor for the poll
 *   operation.  If fds and sem are non-null, then the poll is being setup.
 *   if fds and sem are NULL, then the poll is being torn down.
 *
 ****************************************************************************/
static int poll_fdsetup(int fd, struct pollfd *fds, bool setup)
{
    int iRet = -1;
	DriverDescreptor_t *pTmpDriverDesc = (DriverDescreptor_t *)fd;

  /* Check for a valid file descriptor */

#if 0
  目前没有网络功能
  if ((unsigned int)fd >= CONFIG_NFILE_DESCRIPTORS)
    {
      /* Perform the socket ioctl */

#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0
      if ((unsigned int)fd < (CONFIG_NFILE_DESCRIPTORS+CONFIG_NSOCKET_DESCRIPTORS))
        {
          return net_poll(fd, fds, setup);
        }
      else
#endif
        {
          return -EBADF;
        }
    }
#endif

	if(pTmpDriverDesc->pxDriverOps->poll != NULL)
	{
        xSemaphoreTake(pTmpDriverDesc->xPollMutex, portMAX_DELAY);
		iRet = pTmpDriverDesc->pxDriverOps->poll(&pTmpDriverDesc->xHandle, fds, setup);
        xSemaphoreGive(pTmpDriverDesc->xPollMutex);
	}
	else
	{
		Print_Err("Invalid Operation\n");
		return -1;	
	}

    return iRet;
}

/****************************************************************************
 * Name: poll_setup
 *
 * Description:
 *   Setup the poll operation for each descriptor in the list.
 *
 ****************************************************************************/
static inline int poll_setup(struct pollfd *fds, unsigned int nfds, SemaphoreHandle_t sem)
{
    int ret;
    int i;

  /* Process each descriptor in the list */

  for (i = 0; i < nfds; i++)
    {
      /* Setup the poll descriptor */

      fds[i].sem     = sem;
      fds[i].revents = 0;
      fds[i].priv    = NULL;

      /* Check for invalid descriptors. "If the value of fd is less than 0,
       * events shall be ignored, and revents shall be set to 0 in that entry
       * on return from poll()."
       *
       * NOTE:  There is a potential problem here.  If there is only one fd
       * and if it is negative, then poll will hang.  From my reading of the
       * spec, that appears to be the correct behavior.
       */

      if (fds[i].fd != 0)
        {
          /* Set up the poll on this valid file descriptor */
          /* 由驱动去完成自己的pollset */
          ret = poll_fdsetup(fds[i].fd, &fds[i], true);
          if(ret < 0)
          {
            return ret;
          }  
        }
     }
    return 0;
}

/****************************************************************************
 * Name: poll_teardown
 *
 * Description:
 *   Teardown the poll operation for each descriptor in the list and return
 *   the count of non-zero poll events.
 *
 ****************************************************************************/
static inline int poll_teardown(struct pollfd *fds, unsigned int nfds, int *count)
{
  int status;
  int ret = 0;
  int i;

  /* Process each descriptor in the list */

  *count = 0;
  for (i = 0; i < nfds; i++)
    {
      /* Ignore negative descriptors */

      if (fds[i].fd != 0)
        {
          /* Teardown the poll */

          status = poll_fdsetup(fds[i].fd, &fds[i], false);
          if (status < 0)
            {
              ret = status;
            }
        }

      /* Check if any events were posted */

      if (fds[i].revents != 0)
        {
          (*count)++;
        }

      /* Un-initialize the poll structure */

      fds[i].sem = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: poll
 *
 * Description:
 *   poll() waits for one of a set of file descriptors to become ready to
 *   perform I/O.  If none of the events requested (and no error) has
 *   occurred for any of  the  file  descriptors,  then  poll() blocks until
 *   one of the events occurs.
 *
 * Inputs:
 *   fds  - List of structures describing file descriptors to be monitored
 *   nfds - The number of entries in the list
 *   timeout - Specifies an upper limit on the time for which poll() will
 *     block in milliseconds.  A negative value of timeout means an infinite
 *     timeout.
 *
 * Return:
 *   On success, the number of structures that have nonzero revents fields.
 *   A value of 0 indicates that the call timed out and no file descriptors
 *   were ready.  On error, -1 is returned, and errno is set appropriately:
 ****************************************************************************/
int poll(struct pollfd *fds, unsigned int nfds, int timeout)
{
    xTimerHandle timout_timer;
    SemaphoreHandle_t sem;
	int count = 0;
	int ret;

    sem = xSemaphoreCreateBinary();//xSemaphoreCreateCounting(20, 0);//最大计数值20
	ret = poll_setup(fds, nfds, sem);//所有fd用同一把锁
	if (ret >= 0)
    {
        if(timeout == 0)
            timeout = 1;

	    if (timeout >= 0)
        {
            /* Wait for the poll event with a timeout.  Note that the
             * millisecond timeout has to be converted to system clock
             * ticks for wd_start
             */
            //        Print_Info("Create timer\n");
            timout_timer = xTimerCreate("poll timer", timeout/portTICK_RATE_MS, pdFALSE, NULL, poll_timeout, (void *)sem);
            //      Print_Info("Create timer over\n");
            xTimerStart(timout_timer, portMAX_DELAY);
            //    Print_Info("Timer start and lock\n");
            xSemaphoreTake(sem, portMAX_DELAY);//等待解锁
            //  Print_Info("Delete timer\n");
            xTimerDelete(timout_timer, portMAX_DELAY);
        }
	    else
        {
            /* Wait for the poll event with no timeout */

            xSemaphoreTake(sem, portMAX_DELAY);//上锁
        }

	    /* Teardown the poll operation and get the count of events */

	    ret = poll_teardown(fds, nfds, &count);
    }

	vSemaphoreDelete(sem);

	/* Check for errors */

	if (ret < 0)
	  {
	    return -1;
	  }

	return count;
}

int lseek(int iFd, off_t xOffset, int iWhence)
{
    int iRet = -1;
	DriverDescreptor_t *pTmpDriverDesc = (DriverDescreptor_t *)iFd;
    
    if(iFd == -1 && pTmpDriverDesc->pxDriverOps == NULL)
	{
		Print_Err("Invalid Operation\n");
		goto END;
	}
    
    xSemaphoreTake(pTmpDriverDesc->xSeekMutex, portMAX_DELAY);

	if(pTmpDriverDesc->pxDriverOps->lseek != NULL)
	{
		iRet = pTmpDriverDesc->pxDriverOps->lseek(&pTmpDriverDesc->xHandle, xOffset, iWhence);
	}
	else
	{
		Print_Err("Invalid Operation\n");
		goto END;
	}

END:
	xSemaphoreGive(pTmpDriverDesc->xSeekMutex);
	return iRet;
}

