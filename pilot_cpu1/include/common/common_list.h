#ifndef _COMMON_LIST_H_
#define _COMMON_LIST_H_
//from linux

#ifdef __cplusplus
extern "C"{
#endif

#include <stdio.h>
#include <stdlib.h>

#define LIST_POISON1  ((void *) 0x00100100)
#define LIST_POISON2  ((void *) 0x00200200)

//计算member在type中的位置
//#define offsetof(type, member)  (size_t)(&((type*)0)->member)
//根据member的地址获取type的起始地址
#define container_of(ptr, type, member) ({			\
	const typeof(((type *)0)->member)*__mptr = (ptr); \
	(type *)((char *)__mptr - offsetof(type, member)); })

struct list_head{
	struct list_head *next, *prev;
};

//初始化链表
static inline void INIT_LIST_HEAD(struct list_head *list)
{
	list->next = list;
	list->prev = list;
}

static inline void __list_add(struct list_head *new_node,
			      struct list_head *prev,
			      struct list_head *next)
{
	next->prev = new_node;
	new_node->next = next;
	new_node->prev = prev;
	prev->next = new_node;
}

//从头部添加节点
static inline void list_add(struct list_head *new_node, struct list_head *head)
{
	__list_add(new_node, head, head->next);
}

//从尾部添加节点
static inline void list_add_tail(struct list_head *new_node, struct list_head *head)
{
	__list_add(new_node, head->prev, head);
}

//添加到某个节点的后面
static inline void list_add_after(struct list_head *new_node, struct list_head *prev, struct list_head *head)
{
	//链表尾
	if(head->prev == prev)
	{
		list_add_tail(new_node, head);
	}
	else
	{
		__list_add(new_node, prev, prev->next);
	}
}

static inline void __list_del(struct list_head * prev, struct list_head * next)
{
	next->prev = prev;
	prev->next = next;
}

//删除一个节点
static inline void list_del(struct list_head *entry)
{
    if(entry->prev && entry->next)
    {
        __list_del(entry->prev, entry->next);
        entry->next = (struct list_head *)LIST_POISON1;
        entry->prev = (struct list_head *)LIST_POISON2;
    }
}

static inline int list_empty(const struct list_head *head)
{
	return head->next == head;
}

//获得节点所在的结构体
#define list_entry(ptr, type, member) \
	container_of(ptr, type, member)

//获得链表第一个节点所在的结构体
#define list_first_entry(ptr, type, member) \
	list_entry((ptr)->next, type, member)

//获得链表最后一个节点所在的结构体
#define list_last_entry(ptr, type, member) \
	list_entry((ptr)->prev, type, member)

//获得链表第一个节点所在的结构体，如果链表为空，就返回null
#define list_first_entry_or_null(ptr, type, member) \
	(!list_empty(ptr) ? list_first_entry(ptr, type, member) : NULL)

//获得下一个节点所在的结构体
#define list_next_entry(pos, member) \
	list_entry((pos)->member.next, typeof(*(pos)), member)

//获得前一个节点所在的结构体
#define list_prev_entry(pos, member) \
	list_entry((pos)->member.prev, typeof(*(pos)), member)

//遍历链表
#define list_for_each(pos, head) \
	for (pos = (head)->next; pos != (head); pos = pos->next)

//安全遍历，防止遍历时删除节点导致的死循环
#define list_for_each_safe(pos, n, head) \
	for (pos = (head)->next, n = pos->next; pos != (head); \
		pos = n, n = pos->next)

#define list_for_each_entry(pos, head, member)				\
	for (pos = list_first_entry(head, typeof(*pos), member);	\
	     &pos->member != (head);					\
	     pos = list_next_entry(pos, member))

#define list_for_each_entry_safe(pos, n, head, member)			\
	for (pos = list_first_entry(head, typeof(*pos), member),	\
		n = list_next_entry(pos, member);			\
	     &pos->member != (head); 					\
	     pos = n, n = list_next_entry(n, member))


#ifdef __cplusplus
}
#endif
#endif
