#include "common_list.h"
#include "FreeRTOS_Print.h"

static struct list_head xTestListHead;//链表头

static struct xListTest_t{
	struct list_head xListNode;
	int iNum;
};

void List_test()
{
	struct xListTest_t *xNewListNode;
	struct xListTest_t *xIndex;
	struct xListTest_t *xNext;

	//初始化驱动队列
	INIT_LIST_HEAD(&xTestListHead);	

	
	xNewListNode = (struct xListTest_t *)pvPortMalloc(sizeof(struct xListTest_t));
	xNewListNode->iNum = 0;
	list_add_after(&xNewListNode->xListNode, &xTestListHead, &xTestListHead);
	

	xNewListNode = (struct xListTest_t *)pvPortMalloc(sizeof(struct xListTest_t));
	xNewListNode->iNum = 2;

	list_for_each_entry(xIndex, &xTestListHead, xListNode)
	{
		xNext = list_next_entry(xIndex, xListNode);
		//找到num比当前节点大的，或者到链表尾了，就加到它后面
		if((xNext->iNum > xNewListNode->iNum) || (&xIndex->xListNode == xTestListHead.prev))
		{
			list_add_after(&xNewListNode->xListNode, &xIndex->xListNode, &xTestListHead);
			break;
		}
		
	}

	list_for_each_entry(xIndex, &xTestListHead, xListNode)
	{
		Print_Warn("Current node:%d\n", xIndex->iNum);
	}

	xNewListNode = (struct xListTest_t *)pvPortMalloc(sizeof(struct xListTest_t));
	xNewListNode->iNum = 1;

	list_for_each_entry(xIndex, &xTestListHead, xListNode)
	{
		xNext = list_next_entry(xIndex, xListNode);
		//找到num比当前节点大的，或者到链表尾了，就加到它后面
		if((xNext->iNum > xNewListNode->iNum) || (&xIndex->xListNode == xTestListHead.prev))
		{
			list_add_after(&xNewListNode->xListNode, &xIndex->xListNode, &xTestListHead);
			break;
		}
		
	}

	list_for_each_entry(xIndex, &xTestListHead, xListNode)
	{
		Print_Warn("Current node:%d\n", xIndex->iNum);
	}
}
