#include <fs/fs.h>
#include "task.h"

/* File descriptors ***********************************************************/
struct filelist tg_filelist = {0};      /* All task share one file list */

FAR struct filelist *sched_getfiles(void)
{
    if((void *)tg_filelist.fl_sem == NULL)
        files_initlist(&tg_filelist);

    return &tg_filelist;
}


