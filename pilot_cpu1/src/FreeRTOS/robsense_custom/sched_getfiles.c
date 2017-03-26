#include <fs/fs.h>
#include "task.h"

/*
 * (hebin bin.he@robsense.com)
 */

FAR struct filelist *sched_getfiles(void)
{

    #if(configROBSENSE_CUSTOM == 1)
      return xTaskGetCurrentTaskFileList();
    #endif

      return NULL;
}


